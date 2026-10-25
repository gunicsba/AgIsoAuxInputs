/*
 * Joystick AUX-N - ESP32S3R8N8 CAN Board V1.0.0
 *
 * Real ISO 11783-6 AUX-N (Auxiliary Control Type 2) joystick, built on the
 * AgIsoStack-plus-plus ISOBUS library. It can plug into any ISO 11783
 * network (a tractor's VT, or AgOpenGPS's own VT) and have its inputs
 * assigned to implement functions through the VT's normal aux-assignment
 * screen.
 *
 * Object pool: one Working Set, a main Data Mask drawing the handgrip (each
 * button highlights while pressed; the firmware commit is shown at the
 * bottom), a "Lights" Data Mask controlling the addressable LEDs on GPIO7
 * (see lights.cpp) with soft keys to switch between them, plus 20 Auxiliary
 * Input Type 2 objects (4 analog channels from the ADS1115, 16 buttons from
 * the extender PCA9555).
 * The pool binary is built offline by tools/build_aux_pool.py, which emits
 * both a standalone .iop file (for reference/upload tools) and the embedded
 * C byte array in object_pool/aux_n_pool_data.h that this file actually
 * compiles in.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "esp_log.h"

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_parameter_group_number_request_protocol.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include "git_version.h"
#include "aux_buttons.h"
#include "debug_log.h"
#include "lights.h"
#include "object_pool/aux_n_pool_data.h"
#include "object_pool_adapt.h"
#include "settings.h"
#include "object_pool/object_pool_ids.h"
#include "canlink_bridge.h"
#include "wifi_setup.h"

#if defined(ENABLE_OTA_WEBSERVER)
#include <ESP2SOTA.h>
#include <WebServer.h>
#include <WiFi.h>
#endif

#if defined(ENABLE_SEEDER_COMBO)
#include "object_pool/seeder_pool_parts.h"
#include "seeder_app.hpp"
#include "storage.hpp"
#include "esp32s3/spiram.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#endif

// I2C pins
#define I2C_SDA 39
#define I2C_SCL 38

// CAN pins (TWAI)
#define CAN_TX_PIN 10
#define CAN_RX_PIN 11

// LED pins
#define LED1_PIN 48
#define LED2_PIN 47

// PCA9555 addresses
#define PCA9555_MAIN_ADDR 0x20
// Extender address is auto-detected at boot (0x21-0x23, A0/A1/A2 strapping)
static uint8_t pcaExtAddr = 0;

// PCA9555 registers
#define PCA9555_REG_INPUT0 0x00
#define PCA9555_REG_INPUT1 0x01
#define PCA9555_REG_OUTPUT0 0x02
#define PCA9555_REG_OUTPUT1 0x03
#define PCA9555_REG_CONFIG0 0x06
#define PCA9555_REG_CONFIG1 0x07

// A simple stderr/stdout logger sink for the CAN stack (mirrors the
// library's own console_logger.cpp example, minus stdout timestamps).
class ArduinoCANStackLogger : public isobus::CANStackLogger {
public:
  void sink_CAN_stack_log(CANStackLogger::LoggingLevel level,
                           const std::string &text) override {
    // Debug level is chatty (every TP session...): serial only, so the
    // /debug log keeps the VT handshake, warnings and errors.
#if defined(ENABLE_SEEDER_COMBO)
    // Both working sets share the VT partner, so each VT client also sees
    // (and rejects) every VT message to the other one
    if (text == "[VT]: VT-ECU Client message invalid")
      level = LoggingLevel::Debug;
#endif
    if (level >= LoggingLevel::Info)
      debug_log("%s", text.c_str());
    else
      Serial.println(text.c_str());
  }
};
static ArduinoCANStackLogger canStackLogger;

#if defined(ENABLE_CANLINK_BRIDGE)
using BaseTWAIPlugin = canlink_bridge::BridgedTWAIPlugin;
#else
using BaseTWAIPlugin = isobus::TWAIPlugin;
#endif

#if defined(ENABLE_SEEDER_COMBO)
// TEST: traces our outgoing ETP sessions (ISO 11783-3 5.11) from the raw
// frames, to see whether a slow pool upload advances or the VT keeps asking
// for the same packets again: a CTS whose next packet number goes backwards
// is a retransmit request.
namespace etp_trace {
constexpr uint8_t PF_CM = 0xC8; // ETP.CM, PGN 0xC800
constexpr uint8_t PF_DT = 0xC7; // ETP.DT, PGN 0xC700
constexpr uint8_t RTS = 0x14, CTS = 0x15, DPO = 0x16, EOMA = 0x17, ABORT = 0xFF;
constexpr uint32_t PROGRESS_LOG_PACKETS = 1000;

struct Session {
  uint8_t source = 0xFF; // our address
  uint8_t destination = 0xFF;
  bool active = false;
  uint32_t bytes = 0, packets = 0;
  uint32_t highestNext = 0; // highest next packet number the receiver asked for
  uint32_t ctsCount = 0, holdCount = 0, retransmits = 0;
  uint32_t dataFrames = 0;
  uint32_t startMs = 0, lastCtsMs = 0, longestCtsGapMs = 0;
  uint32_t lastProgressLog = 0;
  uint32_t completed = 0, aborted = 0;
};
static Session sessions[2]; // one per working set on this board

static Session *session_for(uint8_t source) {
  for (Session &s : sessions)
    if (s.source == source)
      return &s;
  for (Session &s : sessions)
    if (s.source == 0xFF) {
      s.source = source;
      return &s;
    }
  return nullptr;
}

static uint32_t u24(const uint8_t *d) { return d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16); }
static uint32_t u32(const uint8_t *d) { return u24(d) | ((uint32_t)d[3] << 24); }

// A frame we transmit
static void on_tx(const isobus::CANMessageFrame &f) {
  if (!f.isExtendedFrame || f.dataLength < 8)
    return;
  const uint8_t pf = (f.identifier >> 16) & 0xFF;
  if (pf != PF_CM && pf != PF_DT)
    return;
  Session *s = session_for(f.identifier & 0xFF);
  if (!s)
    return;
  if (pf == PF_DT) {
    s->dataFrames++;
    return;
  }
  if (f.data[0] == RTS) {
    const uint32_t now = millis();
    const uint32_t completed = s->completed, aborted = s->aborted;
    *s = Session();
    s->source = f.identifier & 0xFF;
    s->completed = completed;
    s->aborted = aborted;
    s->destination = (f.identifier >> 8) & 0xFF;
    s->active = true;
    s->bytes = u32(&f.data[1]);
    s->packets = (s->bytes + 6) / 7;
    s->startMs = s->lastCtsMs = now;
    debug_log("[ETP] %u->%u RTS %lu bytes, %lu packets, PGN 0x%05lX", s->source, s->destination,
              (unsigned long)s->bytes, (unsigned long)s->packets, (unsigned long)u24(&f.data[5]));
  } else if (f.data[0] == ABORT && s->active) {
    s->active = false;
    s->aborted++;
    debug_log("[ETP] %u->%u we aborted (reason %u) at packet %lu of %lu", s->source, s->destination,
              f.data[1], (unsigned long)s->highestNext, (unsigned long)s->packets);
  }
}

// A frame we receive (from the bus or the CANlink client)
static void on_rx(const isobus::CANMessageFrame &f) {
  if (!f.isExtendedFrame || f.dataLength < 8 || ((f.identifier >> 16) & 0xFF) != PF_CM)
    return;
  const uint8_t destination = (f.identifier >> 8) & 0xFF;
  Session *s = nullptr;
  for (Session &candidate : sessions)
    if (candidate.source == destination)
      s = &candidate;
  if (!s || !s->active)
    return;
  const uint32_t now = millis();
  switch (f.data[0]) {
    case CTS: {
      const uint8_t count = f.data[1];
      const uint32_t next = u24(&f.data[2]);
      s->ctsCount++;
      s->longestCtsGapMs = max(s->longestCtsGapMs, now - s->lastCtsMs);
      s->lastCtsMs = now;
      if (0 == count) {
        s->holdCount++; // receiver asks to wait
      } else if (next < s->highestNext) {
        s->retransmits++;
        if (s->retransmits <= 5)
          debug_log("[ETP] %u->%u CTS asks again from packet %lu (had %lu), %u packets", s->source,
                    s->destination, (unsigned long)next, (unsigned long)s->highestNext, count);
      }
      if (next > s->highestNext)
        s->highestNext = next;
      if (s->highestNext - s->lastProgressLog >= PROGRESS_LOG_PACKETS) {
        s->lastProgressLog = s->highestNext;
        debug_log("[ETP] %u->%u at packet %lu of %lu after %lu s, %u per CTS, longest CTS gap %lu ms, "
                  "%lu retransmit requests, %lu holds",
                  s->source, s->destination, (unsigned long)next, (unsigned long)s->packets,
                  (unsigned long)((now - s->startMs) / 1000), count,
                  (unsigned long)s->longestCtsGapMs, (unsigned long)s->retransmits,
                  (unsigned long)s->holdCount);
      }
      break;
    }
    case EOMA:
      s->active = false;
      s->completed++;
      debug_log("[ETP] %u->%u done: %lu bytes in %lu s, %lu CTS, longest gap %lu ms, %lu retransmit "
                "requests, %lu data frames sent",
                s->source, s->destination, (unsigned long)u32(&f.data[1]),
                (unsigned long)((now - s->startMs) / 1000), (unsigned long)s->ctsCount,
                (unsigned long)s->longestCtsGapMs, (unsigned long)s->retransmits,
                (unsigned long)s->dataFrames);
      break;
    case ABORT:
      s->active = false;
      s->aborted++;
      debug_log("[ETP] %u->%u receiver aborted (reason %u) at packet %lu of %lu", s->source,
                s->destination, f.data[1], (unsigned long)s->highestNext, (unsigned long)s->packets);
      break;
    default:
      break;
  }
}

static void debug_status(String &out) {
  out += "ETP uploads (ours)\n";
  for (const Session &s : sessions) {
    if (s.source == 0xFF)
      continue;
    out += "  " + String(s.source) + "->" + String(s.destination) + ": " +
           (s.active ? "active" : "idle") + ", packet " + String(s.highestNext) + " of " +
           String(s.packets) + " (" + String(s.bytes) + " bytes), " + String(s.ctsCount) +
           " CTS, longest gap " + String(s.longestCtsGapMs) + " ms, " + String(s.retransmits) +
           " retransmit requests, " + String(s.holdCount) + " holds, " + String(s.dataFrames) +
           " data frames; " + String(s.completed) + " done, " + String(s.aborted) + " aborted\n";
  }
}
} // namespace etp_trace
#endif

// TWAIPlugin with a minimum gap between transmitted frames. Back-to-back
// TP data frames during an object pool upload drove the bus to bus-off on
// every attempt (most likely the VT's CAN adapter overrunning and answering
// with error frames). A write refused here stays queued in the stack and is
// retried on the next update().
class PacedTWAIPlugin : public BaseTWAIPlugin {
public:
  using BaseTWAIPlugin::BaseTWAIPlugin;

  bool write_frame(const isobus::CANMessageFrame &canFrame) override {
    uint32_t now = micros();
    if (now - lastTxMicros < MIN_TX_GAP_US)
      return false;
    if (!BaseTWAIPlugin::write_frame(canFrame))
      return false;
    lastTxMicros = now;
#if defined(ENABLE_SEEDER_COMBO)
    etp_trace::on_tx(canFrame);
#endif
    return true;
  }

#if defined(ENABLE_SEEDER_COMBO)
  bool read_frame(isobus::CANMessageFrame &canFrame) override {
    if (!BaseTWAIPlugin::read_frame(canFrame))
      return false;
    etp_trace::on_rx(canFrame);
    return true;
  }
#endif

private:
  static constexpr uint32_t MIN_TX_GAP_US = 2000;
  uint32_t lastTxMicros = 0;
};

static Adafruit_ADS1115 ads;
static std::shared_ptr<isobus::VirtualTerminalClient> vtClient = nullptr;
static std::shared_ptr<isobus::InternalControlFunction> internalECU = nullptr;
static std::shared_ptr<isobus::PartneredControlFunction> partnerVT = nullptr;

// Mutable RAM copy of the selected pool: set_object_pool() reads from this
// pointer lazily as the pool uploads, so adaptPoolToVt() (once the connected
// VT is known) has to patch a writable buffer.
static uint8_t poolBuffer[AUX_N_POOL_MAX_SIZE];
static bool poolMultiPage = false;
static bool poolAdapted = false;
// What the VT reported in the current connection handshake (0 = not yet).
// AgIsoStack's getters still hold the previous VT's values until its own
// responses arrive, so the pool is fitted from these instead.
static uint16_t vtMaskSize = 0;
static uint8_t vtKeyWidth = 0;
static uint8_t vtKeyHeight = 0;
// Raw version byte from the VT's Get Memory response, -1 until it arrives
static int vtVersionByte = -1;

static uint16_t analogValues[NUM_AUX_ANALOG] = {0};
static bool buttonStates[NUM_AUX_BUTTONS] = {false}; // for the log / OTA page

static uint32_t lastPollTime = 0;
static uint32_t lastForceResync = 0;
static constexpr uint32_t POLL_INTERVAL_MS = 40;      // ~25 Hz
static constexpr uint32_t FORCE_RESYNC_MS = 1000;      // resend unchanged values periodically
static constexpr uint16_t ANALOG_HYSTERESIS = 48;      // ~0.15% of full 16-bit range, filters ADC noise

void pca9555_write(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t pca9555_read(uint8_t addr, uint8_t reg) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
      continue;
    if (Wire.requestFrom(addr, (uint8_t)1) == 1)
      return Wire.read();
  }
  return 0xFF;
}

// A (re)connected VT shows the pool's initial values: fill in the firmware
// commit. (Button highlights are restored by aux_buttons_update().)
static void syncCommitOnConnect() {
  static bool wasConnected = false;
  bool connected = vtClient->get_is_connected();
  if (connected && !wasConnected) {
    char commit[COMMIT_TEXT_LENGTH + 1];
    snprintf(commit, sizeof(commit), "%*s", COMMIT_TEXT_LENGTH,
             GIT_COMMIT_SHORT); // right-aligned, padded to the pool length
    vtClient->send_change_string_value(STR_COMMIT, commit);
  }
  wasConnected = connected;
}

static void pollInputsAndReport(bool forceResync) {
  // 4 ADS1115 channels -> analog AUX-N inputs (maintains-position pots)
  for (uint8_t ch = 0; ch < NUM_AUX_ANALOG; ch++) {
    int16_t raw = ads.readADC_SingleEnded(ch);
    // ADS1115 single-ended reads are 0..~32767 at GAIN_ONE. AUX-N analog
    // inputs report 0-100% as 0..0xFAFF (ISO 11783-6 Table J.5, J.7.9);
    // above that are the reserved, error and not-available ranges.
    constexpr uint32_t ADC_MAX = 32767;
    constexpr uint32_t AUX_ANALOG_MAX = 0xFAFF;
    uint16_t scaled = (uint16_t)((uint32_t)constrain(raw, (int16_t)0, (int16_t)ADC_MAX) *
                                 AUX_ANALOG_MAX / ADC_MAX);
    uint16_t prev = analogValues[ch];
    if (forceResync || (uint16_t)abs((int32_t)scaled - (int32_t)prev) >
                            ANALOG_HYSTERESIS) {
      analogValues[ch] = scaled;
      vtClient->update_auxiliary_input(AUX_ANALOG_IDS[ch], scaled, 0xFFFF);
    }
  }

  // 16 extender PCA9555 buttons -> momentary boolean AUX-N inputs (single
  // or multi-page, see aux_buttons.cpp)
  if (pcaExtAddr != 0) {
    uint8_t port0 = ~pca9555_read(pcaExtAddr, PCA9555_REG_INPUT0);
    uint8_t port1 = ~pca9555_read(pcaExtAddr, PCA9555_REG_INPUT1);
    uint16_t raw = (uint16_t)port0 | ((uint16_t)port1 << 8);
    aux_buttons_report(raw, forceResync);
    bool anyChanged = false;
    for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
      bool pressed = (raw & (1u << i)) != 0;
      if (pressed != buttonStates[i]) {
        buttonStates[i] = pressed;
        anyChanged = true;
      }
    }
    if (anyChanged) {
      char bits[NUM_AUX_BUTTONS + 1];
      for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
        bits[i] = buttonStates[i] ? '1' : '0';
      }
      bits[NUM_AUX_BUTTONS] = '\0';
      Serial.printf("[BTN] %s\n", bits);
    }
  }
}

static void detectExtenderPCA9555() {
  for (uint8_t addr = 0x21; addr <= 0x23; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      pcaExtAddr = addr;
      pca9555_write(addr, PCA9555_REG_CONFIG0, 0xFF); // inputs
      pca9555_write(addr, PCA9555_REG_CONFIG1, 0xFF);
      pca9555_write(addr, PCA9555_REG_OUTPUT0, 0xFF); // pull-ups
      pca9555_write(addr, PCA9555_REG_OUTPUT1, 0xFF);
      Serial.printf("Extender PCA9555 OK @ 0x%02X\n", addr);
      return;
    }
  }
  Serial.println("WARNING: no extender PCA9555 found at 0x21-0x23");
}

static void logVtHardware(const isobus::CANMessage &message) {
  static int loggedFeatures = -1;
  const uint8_t features = message.get_uint8_at(3);
  if (features == loggedFeatures)
    return;
  loggedFeatures = features;
  // ISO 11783-6 Get Hardware response, hardware features bits 0-7
  static const char *const FEATURE_NAMES[] = {
      "touchscreen",      "pointing device",   "multi-frequency audio", "adjustable volume",
      "simultaneous soft keys", "simultaneous buttons", "drag",       "drag coordinates"};
  String names;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (features & (1u << bit)) {
      if (names.length())
        names += ", ";
      names += FEATURE_NAMES[bit];
    }
  }
  debug_log("[AUX-N] VT hardware: graphic type %u, features 0x%02X (%s)",
            (unsigned)message.get_uint8_at(2), (unsigned)features,
            names.length() ? names.c_str() : "no touchscreen or pointing device");
}

// What the operator does on the VT, as the VT reports it - to tell a press
// that never arrives from one the firmware ignores.
static void logVtInputEvent(const isobus::CANMessage &message) {
  constexpr uint8_t SOFT_KEY_ACTIVATION = 0x00;
  constexpr uint8_t BUTTON_ACTIVATION = 0x01;
  constexpr uint8_t POINTING_EVENT = 0x02;
  constexpr uint8_t SELECT_INPUT_OBJECT = 0x03;
  constexpr uint8_t ESC = 0x04;
  constexpr uint8_t CHANGE_NUMERIC_VALUE = 0x05;
  constexpr uint8_t STILL_HELD = 2;

  switch (message.get_uint8_at(0)) {
    case SOFT_KEY_ACTIVATION:
    case BUTTON_ACTIVATION:
      if (message.get_uint8_at(1) == STILL_HELD)
        return; // repeats while held
      debug_log("[VT-IN] %s %u: code %u (0 released, 1 pressed, 3 aborted), parent %u, key %u",
                message.get_uint8_at(0) == SOFT_KEY_ACTIVATION ? "soft key" : "button",
                (unsigned)message.get_uint16_at(2), (unsigned)message.get_uint8_at(1),
                (unsigned)message.get_uint16_at(4), (unsigned)message.get_uint8_at(6));
      break;
    case POINTING_EVENT:
      debug_log("[VT-IN] pointing event at %u,%u", (unsigned)message.get_uint16_at(1),
                (unsigned)message.get_uint16_at(3));
      break;
    case SELECT_INPUT_OBJECT:
      debug_log("[VT-IN] %s object %u",
                message.get_uint8_at(3) == 0x01 ? "selected" : "deselected",
                (unsigned)message.get_uint16_at(1));
      break;
    case ESC:
      debug_log("[VT-IN] ESC on object %u", (unsigned)message.get_uint16_at(1));
      break;
    case CHANGE_NUMERIC_VALUE:
      debug_log("[VT-IN] object %u changed to %lu", (unsigned)message.get_uint16_at(1),
                (unsigned long)message.get_uint32_at(4));
      break;
    default:
      break;
  }
}

// Messages from the VT to us. The Get Memory response starts every
// connection handshake: the pool is fitted again for whatever VT answers
// (see adaptPoolToVt()), from the version and sizes of this handshake.
// AgIsoStack reports VT version bytes 0-2 (VT2 and the pre-standard versions)
// as "unknown", so the raw byte is taken here; the rest is logged for
// diagnosis.
static void onVtToEcuMessage(const isobus::CANMessage &message, void *) {
  constexpr uint8_t GET_MEMORY_RESPONSE = 0xC0;
  constexpr uint8_t GET_NUMBER_OF_SOFT_KEYS_RESPONSE = 0xC2;
  constexpr uint8_t GET_HARDWARE_RESPONSE = 0xC7;
  if (message.get_data_length() < 8 || !internalECU->get_address_valid() ||
      message.get_identifier().get_destination_address() != internalECU->get_address())
    return; // also skips responses to other working sets on the same VT

  switch (message.get_uint8_at(0)) {
    case GET_MEMORY_RESPONSE:
      vtVersionByte = message.get_uint8_at(1);
      vtMaskSize = vtKeyWidth = vtKeyHeight = 0;
      poolAdapted = false;
      break;
    case GET_NUMBER_OF_SOFT_KEYS_RESPONSE:
      vtKeyWidth = message.get_uint8_at(4);
      vtKeyHeight = message.get_uint8_at(5);
      break;
    case GET_HARDWARE_RESPONSE:
      vtMaskSize = message.get_uint16_at(4);
      logVtHardware(message);
      break;
    default:
      logVtInputEvent(message);
      break;
  }
}

// AgIsoStack answers a Request for Address Claim sent to us with our claim
// (InternalControlFunction) and, as no PGN request callback handles PGN
// 60928, also with a NACK. ISO 11783-5 expects only the claim; on an
// AMATRON 3 bus something kept re-requesting. Mark it handled, without an ACK.
static bool onAddressClaimRequest(std::uint32_t, std::shared_ptr<isobus::ControlFunction>,
                                  bool &acknowledge, isobus::AcknowledgementType &, void *) {
  acknowledge = false;
  return true;
}

static void setupIsobus() {
  // Silence the native ESP-IDF driver logs (ANSI colour codes that show up
  // as raw escape-code clutter in most serial monitors) - the TWAI/GPIO
  // driver logs every pin config line at Info level by default. Our own
  // logging below stays untouched.
  esp_log_level_set("gpio", ESP_LOG_WARN);
  esp_log_level_set("twai", ESP_LOG_WARN);

  isobus::CANStackLogger::set_can_stack_logger_sink(&canStackLogger);
  isobus::CANStackLogger::set_log_level(
      isobus::CANStackLogger::LoggingLevel::Debug);

  // Static: TWAIPlugin keeps pointers to these configs.
  static twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  // The default 5-frame queues overflow during an object pool upload while
  // loop() is busy elsewhere (I2C reads, LEDs), dropping the VT's TP
  // flow-control frames until its session times out.
  twaiConfig.rx_queue_len = 64;
  twaiConfig.tx_queue_len = 32;
  static twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
  static twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Zero timeouts: without threads, update() reads one frame per call, and
  // the default 100 ms blocking receive would stall loop() whenever the bus
  // is quiet. A failed non-blocking write stays queued in the stack and is
  // retried on the next update().
  std::shared_ptr<isobus::CANHardwarePlugin> canDriver =
      std::make_shared<PacedTWAIPlugin>(&twaiConfig, &twaiTiming,
                                         &twaiFilter, 0, 0);

  isobus::CANHardwareInterface::set_number_of_can_channels(1);
  isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

  if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid()) {
    Serial.println("[AUX-N] Failed to start TWAI hardware interface!");
  }

  // NAME: Industry Group 2 (agricultural/forestry), Device Class 0,
  // Function 129 (Operator Controls - Machine Specific) - the closest
  // formal NAME identity for an operator-input device per ISO 11783-1
  // Annex E. AUX-N capability itself is signalled by the object pool
  // content (the AuxiliaryInputType2 objects), not this NAME.
  isobus::NAME deviceName(0);
  deviceName.set_arbitrary_address_capable(true);
  deviceName.set_industry_group(2);
  deviceName.set_device_class(0);
  deviceName.set_function_code(129);
  // A separate NAME per page mode: VTs store pools and AUX assignments by
  // input unit NAME + input object ID (AgIsoVirtualTerminal ignores the
  // model identification code). Sharing one NAME let a single-mode
  // assignment (input 206) be restored onto the multi-page input set, where
  // that input doesn't exist - and AgIsoVirtualTerminal hung.
  deviceName.set_identity_number(settings_multi_page() ? 2 : 1);
  deviceName.set_ecu_instance(0);
  deviceName.set_function_instance(0);
  deviceName.set_device_class_instance(0);
  // Placeholder manufacturer code (matches AgIsoStack's own examples). Get a
  // real assigned SAE/AEF manufacturer code before deploying on a real
  // customer ISOBUS network - fine for bench testing against AgIsoVT.
  deviceName.set_manufacturer_code(1407);

  const isobus::NAMEFilter vtFilter(
      isobus::NAME::NAMEParameters::FunctionCode,
      static_cast<uint8_t>(isobus::NAME::Function::VirtualTerminal));
  const std::vector<isobus::NAMEFilter> vtNameFilters = {vtFilter};

  internalECU =
      isobus::CANNetworkManager::CANNetwork.create_internal_control_function(
          deviceName, 0);
  partnerVT =
      isobus::CANNetworkManager::CANNetwork
          .create_partnered_control_function(0, vtNameFilters);
  if (auto requests = internalECU->get_pgn_request_protocol().lock()) {
    requests->register_pgn_request_callback(
        static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::AddressClaim),
        onAddressClaimRequest, nullptr);
  }

  vtClient =
      std::make_shared<isobus::VirtualTerminalClient>(partnerVT, internalECU);
  isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(
      static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::VirtualTerminalToECU),
      onVtToEcuMessage, nullptr);
  // Hash the actual pool content into the version string. A hardcoded
  // literal here would make the VT think nothing changed across firmware
  // updates (per ISO 11783-6, a VT is allowed to skip re-uploading a pool
  // it already has cached under the same version string for this NAME) and
  // silently keep serving its old cached pool.
  // Until the VT is known, the full layout as built; adaptPoolToVt()
  // replaces it (same size) before the version label is sent.
  bool multiPage = settings_multi_page();
  poolMultiPage = multiPage;
  const AuxPoolLayout &layout = aux_pool_layout(multiPage, AUX_POOL_COMPACT_BELOW_MASK);
  memcpy(poolBuffer, layout.data, layout.size);
  vtClient->set_object_pool(
      0, poolBuffer, layout.size,
      isobus::IOPFileInterface::hash_object_pool_to_version(poolBuffer, layout.size));
  Serial.printf("[AUX-N] %s-page pool, %lu bytes\n", multiPage ? "multi" : "single",
                (unsigned long)layout.size);
  for (uint8_t i = 0; i < NUM_AUX_ANALOG; i++) {
    vtClient->add_auxiliary_input_object_id(AUX_ANALOG_IDS[i]);
  }
  aux_buttons_setup(vtClient, internalECU, multiPage);
  vtClient->initialize(true);
}

// Fits the pool to the connected VT (see object_pool_adapt.h) in every
// connection handshake, once the VT's version (Get Memory response), soft
// key size, fonts and data mask size (Get Hardware response, the last of
// them) are known. All arrive before the VT client sends the pool's version
// label and starts the upload, and this runs between processing received
// frames and the VT client's update, so it always lands before either. The
// version label is the hash of the adapted pool, so a VT keeps the version
// that fits it - and a VT whose sizes changed gets a fresh upload.
static void adaptPoolToVt() {
  if (poolAdapted || vtVersionByte < 0 || 0 == vtMaskSize)
    return; // not far enough into the handshake yet
  poolAdapted = true;

  debug_log("[AUX-N] VT version byte %d, soft key designator %ux%u, data mask %ux%u",
            vtVersionByte, (unsigned)vtKeyWidth, (unsigned)vtKeyHeight,
            (unsigned)vtMaskSize, (unsigned)vtMaskSize);
  // Fonts come from the Get Text Font Data response, already parsed by
  // AgIsoStack before the Get Hardware response that triggers this.
  VtDisplay vt = {vtVersionByte, vtMaskSize, vtKeyWidth, vtKeyHeight,
                  1}; // 6x8 always exists
  for (uint8_t size = 1; size < 15; size++) {
    if (vtClient->get_font_size_supported(
            static_cast<isobus::VirtualTerminalClient::FontSize>(size)))
      vt.fontsSupported |= 1u << size;
  }
  const AuxPoolLayout &layout = aux_pool_layout(poolMultiPage, vtMaskSize);
  aux_pool_adapt(poolBuffer, layout, vt);
  vtClient->set_object_pool(
      0, poolBuffer, layout.size,
      isobus::IOPFileInterface::hash_object_pool_to_version(poolBuffer, layout.size));
}

#if defined(ENABLE_SEEDER_COMBO)
// TEST: the seeder controller (github.com/davidbalogh92/Isobus_ESP_seeder_V2)
// as a second working set with its own NAME, address, VT and TC clients.
// It runs on simulated inputs (all-idle HwInputs, its own speed simulation
// setting); settings and counters persist in its own NVS namespace.
static app::Shared seederShared;
static std::unique_ptr<app::SeederApp> seederApp;
static uint32_t lastSeederTick = 0;
static float seederSaveTimer = 0.0f;
static constexpr uint32_t SEEDER_CYCLE_MS = 20; // its control cycle

static void seederSetup() {
  if (!storage::init())
    debug_log("[SEEDER] NVS init failed");
  seeder::Settings settings;
  seeder::Persistent persistent;
  const bool hadSettings = storage::load_settings(settings);
  storage::load_persistent(persistent);
  seederShared.core.load(settings, persistent);

  app::SeederApp::PoolPart parts[SEEDER_POOL_PART_COUNT];
  uint32_t total = 0;
  for (uint8_t i = 0; i < SEEDER_POOL_PART_COUNT; i++) {
    parts[i] = {SEEDER_POOL_PARTS[i], SEEDER_POOL_PART_SIZES[i]};
    total += SEEDER_POOL_PART_SIZES[i];
  }
  // AgIsoStack scales a pool for another VT size in a RAM copy of all its
  // parts - 150 kB, more than the free internal heap. Without PSRAM the pool
  // goes up unscaled (400 px design size), straight from flash.
  const bool scale = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 512 * 1024;
  seederApp = std::make_unique<app::SeederApp>(seederShared);
  // The joystick's VT partner: AgIsoStack binds an address claim to one
  // partner only, so a second partner for the same VT would never be found.
  const bool started = seederApp->start(0, parts, SEEDER_POOL_PART_COUNT, SEEDER_POOL_VERSION,
                                        partnerVT, scale);
  debug_log("[SEEDER] %s: pool %lu bytes in %u parts, version %s, settings %s, %s",
            started ? "started" : "START FAILED", (unsigned long)total,
            (unsigned)SEEDER_POOL_PART_COUNT, SEEDER_POOL_VERSION,
            hadSettings ? "loaded" : "defaults",
            scale ? "scaled to the VT (PSRAM)" : "unscaled (no PSRAM)");
}

// Why PSRAM is (not) there: ESP-IDF only logs a failed boot-time init on
// UART0. The package version says which flash/PSRAM the chip has.
static void psramDebugStatus(String &out) {
  uint32_t pkg = 0;
  esp_efuse_read_field_blob(ESP_EFUSE_PKG_VERSION, &pkg, 3);
  out += "  PSRAM: IDF init " + String(esp_spiram_is_initialized() ? "ok" : "failed") +
         ", chip " + String(esp_spiram_get_size()) + " bytes, heap " +
         String(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)) + " free of " +
         String(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)) + " (Arduino: found " +
         String(psramFound() ? "yes" : "no") + ", " + String(ESP.getPsramSize()) +
         "); chip revision " + String(ESP.getChipRevision()) + ", package " + String(pkg) + "\n";
}

static void seederLoop() {
  const uint32_t now = millis();
  if (now - lastSeederTick < SEEDER_CYCLE_MS)
    return;
  lastSeederTick = now;
  constexpr float CYCLE_S = SEEDER_CYCLE_MS / 1000.0f;
  seederApp->update(CYCLE_S, app::HwInputs{});

  // Same as the seeder's own main loop: settings 2 s after the last change
  bool saveSettings = false, savePersistent = false;
  seeder::Settings settings;
  seeder::Persistent persistent;
  {
    std::lock_guard<std::mutex> guard(seederShared.lock);
    if (seederShared.core.settings_dirty) {
      seederSaveTimer += CYCLE_S;
      if (seederSaveTimer >= 2.0f) {
        seederShared.core.settings_dirty = false;
        seederSaveTimer = 0.0f;
        saveSettings = true;
        settings = seederShared.core.settings();
      }
    }
    if (seederShared.core.persistent_dirty) {
      seederShared.core.persistent_dirty = false;
      savePersistent = true;
      persistent = seederShared.core.persistent();
    }
  }
  if (saveSettings && !storage::save_settings(settings))
    debug_log("[SEEDER] saving settings failed");
  if (savePersistent && !storage::save_persistent(persistent))
    debug_log("[SEEDER] saving counters failed");
}

static void seederDebugStatus(String &out) {
  const app::IsobusDiag d = seederApp->diag();
  out += "Seeder (second working set)\n  address ";
  out += d.address_claimed ? String(d.address) : "not claimed";
  out += ", VT " + String(d.vt_connected ? "connected" : (d.vt_found ? "found" : "not found")) +
         " (" + String(d.vt_width) + "x" + String(d.vt_height) + ", soft key " +
         String(d.vt_softkey_w) + "x" + String(d.vt_softkey_h) + "), page " + d.page.c_str() +
         "\n  pool " + String(SEEDER_POOL_PART_COUNT) + " parts (";
  for (uint8_t i = 0; i < SEEDER_POOL_PART_COUNT; i++)
    out += (i ? " + " : "") + String(SEEDER_POOL_PART_SIZES[i]);
  out += " bytes), version " SEEDER_POOL_VERSION ", VT commands " + String(d.vt_cmd_sent) +
         " sent / " + String(d.vt_cmd_failed) + " failed\n  TC " +
         String(d.tc_connected ? "connected" : (d.tc_found ? "found" : "not found")) +
         ", DDOP uploads " + String(d.ddop_uploads) + "\n";
  psramDebugStatus(out);
}
#endif

static const char *vtVersionName(isobus::VirtualTerminalClient::VTVersion version) {
  switch (version) {
    case isobus::VirtualTerminalClient::VTVersion::Version2OrOlder: return "2 or older";
    case isobus::VirtualTerminalClient::VTVersion::Version3: return "3";
    case isobus::VirtualTerminalClient::VTVersion::Version4: return "4";
    case isobus::VirtualTerminalClient::VTVersion::Version5: return "5";
    case isobus::VirtualTerminalClient::VTVersion::Version6: return "6";
    default: return "unknown";
  }
}

#if defined(ENABLE_OTA_WEBSERVER)
static WebServer otaServer(80);

static void handleOtaRoot() {
  String html = "<html><body style='font-family:sans-serif'>"
                "<h3>Joystick AUX-N</h3><p>ISOBUS AUX-N joystick firmware.</p>"
                "<p><a href='/wifi'>Wi-Fi setup</a> | "
                "<a href='/debug'>Debug</a> | "
                "<a href='/update'>Firmware update</a></p>"
                "<p>Analog: ";
  for (uint8_t i = 0; i < NUM_AUX_ANALOG; i++) {
    html += String(analogValues[i]) + " ";
  }
  html += "</p><p>Buttons: ";
  for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
    html += buttonStates[i] ? "1" : "0";
  }
  html += "</p></body></html>";
  otaServer.send(200, "text/html", html);
}

// Plain text, so it reads the same in a browser and from curl.
static void handleDebug() {
  const uint32_t now = millis();
  String out;
  out.reserve(12000);
  out += "Joystick AUX-N " + String(GIT_COMMIT_SHORT) + ", up " + String(now / 1000) +
         " s, reset reason " + String((int)esp_reset_reason()) + "\n";
  out += "heap free " + String(ESP.getFreeHeap()) + ", min " + String(ESP.getMinFreeHeap()) +
         "\n\nISOBUS\n  address ";
  out += internalECU->get_address_valid() ? String(internalECU->get_address()) : "not claimed";
  out += ", VT " + String(vtClient->get_is_connected() ? "connected" : "not connected") +
         " (version " + vtVersionName(vtClient->get_connected_vt_version()) + ", byte " +
         String(vtVersionByte) + ", soft key " +
         String(vtClient->get_softkey_x_axis_pixels()) + "x" +
         String(vtClient->get_softkey_y_axis_pixels()) + ", data mask " +
         String(vtClient->get_number_x_pixels()) + "x" +
         String(vtClient->get_number_y_pixels()) + "), " +
         (settings_multi_page() ? "multi" : "single") + "-page pool\n";
  twai_status_info_t twai;
  if (twai_get_status_info(&twai) == ESP_OK) {
    static const char *stateNames[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
    out += "  TWAI " + String(twai.state < 4 ? stateNames[twai.state] : "?") + ", tx_err " +
           String(twai.tx_error_counter) + ", rx_err " + String(twai.rx_error_counter) +
           ", bus_err " + String(twai.bus_error_count) + ", msgs_to_tx " +
           String(twai.msgs_to_tx) + "\n";
  }
  out += "\n";
#if defined(ENABLE_SEEDER_COMBO)
  seederDebugStatus(out);
  etp_trace::debug_status(out);
  out += "\n";
#endif
  wifi_debug_status(out);
#if defined(ENABLE_CANLINK_BRIDGE)
  out += "\n";
  canlink_bridge::debug_status(out);
#endif
  out += "\nLog (newest last)\n";
  debug_log_dump(out);
  otaServer.send(200, "text/plain; charset=utf-8", out);
}

static void ota_setup() {
#if defined(ENABLE_CANLINK_BRIDGE)
  wifi_setup(otaServer, canlink_bridge::wifi_settings());
#else
  wifi_setup(otaServer, {"joystick-auxn", "joystick-auxn", "12345678",
                         IPAddress(192, 168, 4, 1), 1});
#endif

  otaServer.on("/", HTTP_GET, handleOtaRoot);
  otaServer.on("/debug", HTTP_GET, handleDebug);
  otaServer.begin();
  ESP2SOTA.begin(&otaServer);
  debug_log("[OTA] Web server started on port 80 (/debug, /update)");
}

static void ota_loop() {
  otaServer.handleClient();
  wifi_update();
}
#endif

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000)
    delay(10);

  Serial.println("\n=== Joystick AUX-N (ISOBUS Auxiliary Control Type 2) ===");
  // esp_reset_reason_t: 1 power-on, 3 software, 4 panic/exception,
  // 5-7 watchdogs, 9 brownout - tells a crash apart from a RESET press.
  debug_log("Commit: %s  Reset reason: %d", GIT_COMMIT_SHORT, (int)esp_reset_reason());

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, HIGH); // on during init
  digitalWrite(LED2_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 NOT found!");
    while (1) {
      delay(100);
    }
  }
  ads.setGain(GAIN_ONE); // +/- 4.096V
  // 860 SPS: each single-shot read blocks ~1.2 ms instead of ~8 ms at the
  // default 128 SPS, so polling 4 channels no longer starves the CAN loop.
  ads.setDataRate(RATE_ADS1115_860SPS);
  Serial.println("ADS1115 OK");

  detectExtenderPCA9555();

  settings_load(); // the page mode picks the object pool
  setupIsobus();
  Serial.println("[AUX-N] ISOBUS stack started, claiming address...");
  lights_setup(vtClient);
  settings_setup(vtClient);
#if defined(ENABLE_SEEDER_COMBO)
  seederSetup();
#endif

#if defined(ENABLE_OTA_WEBSERVER)
  ota_setup(); // Wi-Fi + mDNS, which the bridge needs
#endif
#if defined(ENABLE_CANLINK_BRIDGE)
  canlink_bridge::begin();
#endif

  digitalWrite(LED1_PIN, LOW); // ready
}

static uint32_t lastStatusPrint = 0;
static constexpr uint32_t STATUS_PRINT_MS = 2000;

static void printStatus() {
  twai_status_info_t twaiStatus;
  const char *stateStr = "?";
  if (twai_get_status_info(&twaiStatus) == ESP_OK) {
    static const char *stateNames[] = {"STOPPED", "RUNNING", "BUS_OFF",
                                        "RECOVERING"};
    stateStr = (twaiStatus.state < 4) ? stateNames[twaiStatus.state] : "?";
  } else {
    twaiStatus = {};
  }
  Serial.printf("\n[AUX-N] addr_valid=%d vt_connected=%d pots=%u,%u,%u,%u\n",
                internalECU->get_address_valid(), vtClient->get_is_connected(),
                analogValues[0], analogValues[1], analogValues[2],
                analogValues[3]);
  Serial.printf("[TWAI] state=%s tx_err=%lu rx_err=%lu tx_failed=%lu "
                "arb_lost=%lu bus_err=%lu msgs_to_tx=%lu\n",
                stateStr, (unsigned long)twaiStatus.tx_error_counter,
                (unsigned long)twaiStatus.rx_error_counter,
                (unsigned long)twaiStatus.tx_failed_count,
                (unsigned long)twaiStatus.arb_lost_count,
                (unsigned long)twaiStatus.bus_error_count,
                (unsigned long)twaiStatus.msgs_to_tx);
#if defined(ENABLE_CANLINK_BRIDGE)
  canlink_bridge::print_status();
#endif
}

static constexpr uint8_t CAN_UPDATES_PER_LOOP = 16;

// The TWAI controller stops transmitting for good after bus-off unless the
// application recovers it (ESP-IDF doesn't, and neither does TWAIPlugin).
static void recoverFromBusOff() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK)
    return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.printf("[TWAI] bus-off (bus_err=%lu), recovering\n",
                  (unsigned long)status.bus_error_count);
    twai_initiate_recovery();
  } else if (status.state == TWAI_STATE_STOPPED) {
    // Recovery finished (128 x 11 recessive bits seen): restart.
    twai_start();
    Serial.println("[TWAI] recovered, restarted");
  }
}

void loop() {
  // AgIsoStack compiles out its own background worker threads when ARDUINO
  // is defined (see CANHardwareInterface::update_thread_function() and
  // VirtualTerminalClient::worker_thread_function(), both guarded by
  // `#if !defined CAN_STACK_DISABLE_THREADS && !defined ARDUINO`) - the
  // application has to drive both by calling update() itself, as often as
  // possible: CANHardwareInterface for RX/TX pumping and the network
  // manager (address claiming, ...), VirtualTerminalClient for its own
  // state machine (VT handshake, object pool upload, ...).
  // Each update() moves at most one received frame, so pump several times
  // to keep up with bursts (e.g. TP flow control during the pool upload).
  for (uint8_t i = 0; i < CAN_UPDATES_PER_LOOP; i++) {
    isobus::CANHardwareInterface::update();
  }
  recoverFromBusOff();
  adaptPoolToVt();
  vtClient->update();
  lights_update();
  settings_update();
  aux_buttons_update();
  syncCommitOnConnect();
#if defined(ENABLE_SEEDER_COMBO)
  seederLoop();
#endif

  uint32_t now = millis();
  if (now - lastPollTime >= POLL_INTERVAL_MS) {
    lastPollTime = now;
    bool forceResync = (now - lastForceResync >= FORCE_RESYNC_MS);
    if (forceResync)
      lastForceResync = now;
    pollInputsAndReport(forceResync);
  }
  if (now - lastStatusPrint >= STATUS_PRINT_MS) {
    lastStatusPrint = now;
    printStatus();
  }
#if defined(ENABLE_OTA_WEBSERVER)
  ota_loop();
#endif
  delay(1);
}

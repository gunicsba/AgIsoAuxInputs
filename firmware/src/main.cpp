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
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include "git_version.h"
#include "aux_buttons.h"
#include "debug_log.h"
#include "lights.h"
#include "object_pool/aux_n_pool_data.h"
#include "object_pool/aux_n_pool_justify.h"
#include "object_pool/aux_n_pool_vt2.h"
#include "settings.h"
#include "object_pool/object_pool_ids.h"
#include "canlink_bridge.h"
#include "wifi_setup.h"

#if defined(ENABLE_OTA_WEBSERVER)
#include <ESP2SOTA.h>
#include <WebServer.h>
#include <WiFi.h>
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
    return true;
  }

private:
  static constexpr uint32_t MIN_TX_GAP_US = 2000;
  uint32_t lastTxMicros = 0;
};

static Adafruit_ADS1115 ads;
static std::shared_ptr<isobus::VirtualTerminalClient> vtClient = nullptr;
static std::shared_ptr<isobus::InternalControlFunction> internalECU = nullptr;

// Mutable RAM copy of the selected pool: set_object_pool() reads from this
// pointer lazily as the pool uploads, so the VT4 justification upgrade below
// (once the connected VT's version is known) has to patch a writable buffer.
static uint8_t poolBuffer[AUX_N_POOL_MULTI_SIZE > AUX_N_POOL_SINGLE_SIZE
                             ? AUX_N_POOL_MULTI_SIZE
                             : AUX_N_POOL_SINGLE_SIZE];
static const AuxPoolJustifyUpgrade *poolJustifyUpgrades = nullptr;
static uint32_t poolJustifyUpgradeCount = 0;
static const AuxPoolVt3Object *poolVt3Objects = nullptr;
static uint32_t poolVt3ObjectCount = 0;
static const uint8_t *poolCompactData = nullptr; // same pool, 200x200 layout
static uint32_t poolSize = 0;
static bool poolAdapted = false;
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
    // ADS1115 single-ended reads are 0..~32767 at GAIN_ONE; scale to the
    // full 16-bit range AUX-N analog inputs report.
    uint16_t scaled = (uint16_t)constrain((int32_t)raw * 2, 0, 0xFFFF);
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

// Messages from the VT to us. AgIsoStack reports VT version bytes 0-2 (VT2
// and the pre-standard versions) as "unknown", so take the raw byte from the
// VT's Get Memory response instead; the rest is logged for diagnosis.
static void onVtToEcuMessage(const isobus::CANMessage &message, void *) {
  constexpr uint8_t GET_MEMORY_RESPONSE = 0xC0;
  constexpr uint8_t GET_HARDWARE_RESPONSE = 0xC7;
  if (message.get_data_length() < 8 || !internalECU->get_address_valid() ||
      message.get_identifier().get_destination_address() != internalECU->get_address())
    return; // also skips responses to other working sets on the same VT

  switch (message.get_uint8_at(0)) {
    case GET_MEMORY_RESPONSE:
      if (message.get_uint8_at(1) != vtVersionByte) {
        vtVersionByte = message.get_uint8_at(1);
        debug_log("[AUX-N] VT Get Memory response: version byte %u", (unsigned)vtVersionByte);
      }
      break;
    case GET_HARDWARE_RESPONSE:
      logVtHardware(message);
      break;
    default:
      logVtInputEvent(message);
      break;
  }
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
  auto partnerVT =
      isobus::CANNetworkManager::CANNetwork
          .create_partnered_control_function(0, vtNameFilters);

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
  bool multiPage = settings_multi_page();
  const uint8_t *poolData = multiPage ? AUX_N_POOL_MULTI_DATA : AUX_N_POOL_SINGLE_DATA;
  poolSize = multiPage ? AUX_N_POOL_MULTI_SIZE : AUX_N_POOL_SINGLE_SIZE;
  poolCompactData = multiPage ? AUX_N_POOL_MULTI_COMPACT_DATA : AUX_N_POOL_SINGLE_COMPACT_DATA;
  std::string poolVersion =
      isobus::IOPFileInterface::hash_object_pool_to_version(poolData, poolSize);
  memcpy(poolBuffer, poolData, poolSize);
  poolJustifyUpgrades = multiPage ? AUX_N_POOL_MULTI_JUSTIFY : AUX_N_POOL_SINGLE_JUSTIFY;
  poolJustifyUpgradeCount =
      multiPage ? AUX_N_POOL_MULTI_JUSTIFY_COUNT : AUX_N_POOL_SINGLE_JUSTIFY_COUNT;
  poolVt3Objects = multiPage ? AUX_N_POOL_MULTI_VT3_OBJECTS : AUX_N_POOL_SINGLE_VT3_OBJECTS;
  poolVt3ObjectCount =
      multiPage ? AUX_N_POOL_MULTI_VT3_OBJECTS_COUNT : AUX_N_POOL_SINGLE_VT3_OBJECTS_COUNT;
  vtClient->set_object_pool(0, poolBuffer, poolSize, poolVersion);
  Serial.printf("[AUX-N] %s-page pool, %lu bytes\n", multiPage ? "multi" : "single",
                (unsigned long)poolSize);
  for (uint8_t i = 0; i < NUM_AUX_ANALOG; i++) {
    vtClient->add_auxiliary_input_object_id(AUX_ANALOG_IDS[i]);
  }
  aux_buttons_setup(vtClient, internalECU, multiPage);
  vtClient->initialize(true);
}

// The pools are built for VT3 and a 480x480 data mask, with VT3-safe
// justification, and adapted here once the VT's version (Get Memory
// response) and data mask size (Get Hardware response) are known. Both
// arrive before the VT client sends the pool's version label and starts the
// upload, and this runs between processing received frames and the VT
// client's update, so it always lands before either.
// - Data mask under 480: swap in the compact (200x200) layout, which has the
//   same size and object offsets, under its own version label.
// - VT4+: vertical justification bits are reserved on VT3 and older, and a
//   VT3/Mueller unit rejected the pool outright with them set (see
//   JUSTIFY_HORIZONTAL_MASK in tools/build_aux_pool.py) - patch the full
//   justification back in.
// - VT2 or older: AUX-N objects don't exist yet, and an AMATRON 3 (version
//   byte 2) rejected the pool at the first one. Blank them out into String
//   Variables of the same length (see VT3_OBJECT_TYPES in the builder). The
//   pool still loads, but the VT can't assign the inputs.
static void adaptPoolToVt() {
  uint16_t maskSize = vtClient->get_number_x_pixels();
  if (poolAdapted || vtVersionByte < 0 || 0 == maskSize)
    return; // not connected far enough yet to know the VT
  poolAdapted = true;

  if (maskSize < AUX_N_POOL_LAYOUT_MASK_SIZE) {
    memcpy(poolBuffer, poolCompactData, poolSize);
    vtClient->set_object_pool(
        0, poolBuffer, poolSize,
        isobus::IOPFileInterface::hash_object_pool_to_version(poolCompactData, poolSize));
    debug_log("[AUX-N] VT data mask %u: using the %ux%u layout", (unsigned)maskSize,
              (unsigned)AUX_N_POOL_COMPACT_MASK_SIZE, (unsigned)AUX_N_POOL_COMPACT_MASK_SIZE);
  }

  if (vtVersionByte >= 4 && vtVersionByte != 0xFF) {
    for (uint32_t i = 0; i < poolJustifyUpgradeCount; i++) {
      poolBuffer[poolJustifyUpgrades[i].offset] = poolJustifyUpgrades[i].value;
    }
    Serial.println("[AUX-N] VT reports version 4+: upgraded justification");
  } else if (vtVersionByte < 3 || vtVersionByte == 0xFF) {
    for (uint32_t i = 0; i < poolVt3ObjectCount; i++) {
      uint8_t *object = &poolBuffer[poolVt3Objects[i].offset];
      uint16_t valueLength = poolVt3Objects[i].length - AUX_POOL_STRING_VARIABLE_HEADER;
      object[2] = AUX_POOL_TYPE_STRING_VARIABLE; // keeps its object ID
      object[3] = valueLength & 0xFF;
      object[4] = valueLength >> 8;
      memset(&object[AUX_POOL_STRING_VARIABLE_HEADER], ' ', valueLength);
    }
    debug_log("[AUX-N] VT2 or older: AUX-N not supported, blanked %lu AUX-N objects",
              (unsigned long)poolVt3ObjectCount);
  }
}

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

// Logs what the VT reported about itself once it's known (before the pool
// upload starts), and again only if it changes - a VT that keeps rejecting
// the pool reconnects every few seconds. Pool objects that need to fit the
// soft key designator (keys, AUX input designators) are checked against
// these sizes by a real VT.
static void logVtCapabilities() {
  static uint32_t logged = 0;

  uint8_t keyW = vtClient->get_softkey_x_axis_pixels();
  uint8_t keyH = vtClient->get_softkey_y_axis_pixels();
  uint16_t maskW = vtClient->get_number_x_pixels();
  if (vtVersionByte < 0 || 0 == keyW || 0 == maskW)
    return;
  uint32_t current = ((uint32_t)vtVersionByte << 24) ^ ((uint32_t)keyW << 16) ^
                     ((uint32_t)keyH << 8) ^ maskW;
  if (current == logged)
    return;
  logged = current;
  debug_log("[AUX-N] VT version byte %d, soft key designator %ux%u, data mask %ux%u",
            vtVersionByte, (unsigned)keyW, (unsigned)keyH, (unsigned)maskW,
            (unsigned)vtClient->get_number_y_pixels());
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
  logVtCapabilities();
  lights_update();
  settings_update();
  aux_buttons_update();
  syncCommitOnConnect();

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

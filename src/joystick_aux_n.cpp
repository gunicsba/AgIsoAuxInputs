/*
 * Joystick AUX-N - ESP32S3R8N8 CAN Board V1.0.0
 *
 * Real ISO 11783-6 AUX-N (Auxiliary Control Type 2) joystick, built on the
 * AgIsoStack-plus-plus ISOBUS library. Unlike the other firmware variants in
 * this repo, this speaks genuine ISOBUS, not the project's internal
 * proprietary CAN protocol - it can plug into any ISO 11783 network (a
 * tractor's VT, or AgOpenGPS's own VT) and have its inputs assigned to
 * implement functions through the VT's normal aux-assignment screen.
 *
 * Object pool: one Working Set + one Data Mask showing "TEST" (enough to
 * confirm the pool uploaded), plus 20 Auxiliary Input Type 2 objects (4
 * analog channels from the ADS1115, 16 buttons from the extender PCA9555).
 * The pool binary is built offline by tools/build_aux_pool.py, which emits
 * both a standalone .iop file (for reference/upload tools) and the embedded
 * C byte array in object_pool/aux_n_pool_data.h that this file actually
 * compiles in.
 *
 * Hardware I/O (ADS1115 + PCA9555 reads) is the same driver code used by
 * joystick_full.cpp for the same physical board.
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

#include "object_pool/aux_n_pool_data.h"
#include "object_pool/object_pool_ids.h"

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
    (void)level;
    Serial.println(text.c_str());
  }
};
static ArduinoCANStackLogger canStackLogger;

static Adafruit_ADS1115 ads;
static std::shared_ptr<isobus::VirtualTerminalClient> vtClient = nullptr;
static std::shared_ptr<isobus::InternalControlFunction> internalECU = nullptr;

static uint16_t analogValues[NUM_AUX_ANALOG] = {0};
static bool buttonStates[NUM_AUX_BUTTONS] = {false};
static uint16_t buttonTransitions[NUM_AUX_BUTTONS] = {0};

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

  // 16 extender PCA9555 buttons -> momentary boolean AUX-N inputs
  if (pcaExtAddr != 0) {
    uint8_t port0 = ~pca9555_read(pcaExtAddr, PCA9555_REG_INPUT0);
    uint8_t port1 = ~pca9555_read(pcaExtAddr, PCA9555_REG_INPUT1);
    uint16_t raw = (uint16_t)port0 | ((uint16_t)port1 << 8);
    bool anyChanged = false;
    for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
      bool pressed = (raw & (1u << i)) != 0;
      if (forceResync || pressed != buttonStates[i]) {
        if (pressed != buttonStates[i]) {
          buttonStates[i] = pressed;
          buttonTransitions[i]++;
          anyChanged = true;
        }
        vtClient->update_auxiliary_input(AUX_BUTTON_IDS[i],
                                          buttonStates[i] ? 1 : 0,
                                          buttonTransitions[i]);
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

  twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  std::shared_ptr<isobus::CANHardwarePlugin> canDriver =
      std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming,
                                            &twaiFilter);

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
  deviceName.set_identity_number(1);
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
  // Hash the actual pool content into the version string. A hardcoded
  // literal here would make the VT think nothing changed across firmware
  // updates (per ISO 11783-6, a VT is allowed to skip re-uploading a pool
  // it already has cached under the same version string for this NAME) and
  // silently keep serving its old cached pool.
  std::string poolVersion = isobus::IOPFileInterface::hash_object_pool_to_version(
      AUX_N_POOL_DATA, AUX_N_POOL_SIZE);
  vtClient->set_object_pool(0, AUX_N_POOL_DATA, AUX_N_POOL_SIZE, poolVersion);
  vtClient->set_auxiliary_input_model_identification_code(1);
  for (uint8_t i = 0; i < NUM_AUX_ANALOG; i++) {
    vtClient->add_auxiliary_input_object_id(AUX_ANALOG_IDS[i]);
  }
  for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
    vtClient->add_auxiliary_input_object_id(AUX_BUTTON_IDS[i]);
  }
  vtClient->initialize(true);
}

#if defined(ENABLE_OTA_WEBSERVER)
static WebServer otaServer(80);

static void handleOtaRoot() {
  String html = "<html><body style='font-family:sans-serif'>"
                "<h3>Joystick AUX-N</h3><p>ISOBUS AUX-N joystick firmware.</p>"
                "<p><a href='/update'>Firmware update</a></p>"
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

static void ota_setup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("joystick-auxn", "12345678");
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[OTA] AP started, IP: %s\n", ip.toString().c_str());

  otaServer.on("/", HTTP_GET, handleOtaRoot);
  otaServer.begin();
  ESP2SOTA.begin(&otaServer);
  Serial.println("[OTA] Web server started on port 80 (/update for firmware)");
}

static void ota_loop() { otaServer.handleClient(); }
#endif

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000)
    delay(10);

  Serial.println("\n=== Joystick AUX-N (ISOBUS Auxiliary Control Type 2) ===");

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
  Serial.println("ADS1115 OK");

  detectExtenderPCA9555();

  setupIsobus();
  Serial.println("[AUX-N] ISOBUS stack started, claiming address...");

#if defined(ENABLE_OTA_WEBSERVER)
  ota_setup();
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
  isobus::CANHardwareInterface::update();
  vtClient->update();

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

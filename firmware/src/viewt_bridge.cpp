/*
 * View-T Wi-Fi/CAN bridge - see viewt_bridge.h.
 *
 * Ported from the standalone M5Stack Core2 bridge (ISOBUS_DEMO/
 * proeminon_main.cpp). Differences: the TWAI controller is shared with the
 * ISOBUS stack through BridgedTWAIPlugin, the CAN baud rate is fixed at
 * ISOBUS's 250 kbit/s, and the device ID is this board's own MAC.
 */
#if defined(ENABLE_VIEWT_BRIDGE)

#include "viewt_bridge.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include <errno.h>
#include <lwip/sockets.h>

#include "driver/twai.h"
#include "esp_mac.h"

namespace viewt_bridge {

// View-T values recovered from the APK.
static constexpr char AP_SSID[] = "View-T "; // trailing space is intentional
static constexpr char AP_PASSWORD[] = "12345678";
static const IPAddress AP_IP(192, 168, 0, 34);
static constexpr char HOSTNAME[] = "viewt";
static constexpr uint16_t TCP_PORT = 30000;

static constexpr uint8_t SOF = 0x43;
static constexpr uint8_t EOF_BYTE = 0x0D;
static constexpr uint8_t EXTENDED_TYPE = 0x02;
static constexpr uint8_t OBJECT_DICTIONARY_TYPE = 0xA5;
static constexpr uint8_t OD_READ = 0x00;
static constexpr uint8_t OD_WRITE = 0x01;
static constexpr uint8_t OD_READ_RESPONSE = 0x40;
static constexpr uint32_t CAN_ID_MASK = 0x1FFFFFFF;

// Proemion Byte Command Protocol CAN controller setup commands. ISOBUS is
// always 250 kbit/s and the controller is shared with the stack, so only that
// rate is accepted.
static constexpr uint8_t COMMAND_GET_CAN_BAUD_RATE = 0x56;
static constexpr uint8_t COMMAND_SET_CAN_BAUD_RATE = 0x57;
static constexpr uint8_t COMMAND_DEVICE_BUSY = 0x48;
static constexpr uint8_t CAN_BAUD_250K = 0x04;

// Object dictionary entries used by the View-T app during license activation.
static constexpr uint16_t OD_DEVICE = 0x3007;
static constexpr uint8_t OD_DEVICE_ID = 0x02;
static constexpr uint16_t OD_LICENSE = 0x3009;
static constexpr uint8_t OD_LICENSE_KEY = 0x0C;
static constexpr uint8_t OD_DEMO_EXPIRY = 0x0D;
static constexpr uint16_t OD_SETTINGS = 0x3000;
static constexpr uint8_t OD_WIFI_CHANNEL = 0x0F;
static constexpr size_t OD_VALUE_CAPACITY = 64;

// No PSRAM on this build, so queues are sized for internal RAM. TCP batches
// stay below a normal TCP MSS.
static constexpr UBaseType_t TCP_QUEUE_LENGTH = 512;
static constexpr UBaseType_t STACK_RX_QUEUE_LENGTH = 256;
static constexpr size_t TCP_CAN_BATCH_CAPACITY = 1360;
static constexpr uint32_t TCP_WRITE_TIMEOUT_MS = 500;
// Tablet frames are sent from the TCP task, which can afford to wait briefly
// for room in the TWAI TX queue.
static constexpr TickType_t TABLET_TO_BUS_TIMEOUT = pdMS_TO_TICKS(20);
static constexpr uint32_t TWAI_ERROR_PASSIVE_LIMIT = 128;

struct QueuedTcpFrame {
  uint32_t connectionGeneration;
  uint8_t length;
  uint8_t data[17];
};

static QueueHandle_t tcpQueue = nullptr;     // CAN frames -> tablet
static QueueHandle_t stackRxQueue = nullptr; // bus + tablet frames -> stack
static bool rxTaskStarted = false;

static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;
static Preferences preferences;
static uint8_t tcpBuffer[4096];
static size_t tcpBufferLength = 0;
static char deviceId[18];
static uint8_t licenseKey[OD_VALUE_CAPACITY];
static size_t licenseKeyLength = 0;
static uint8_t demoExpiry[OD_VALUE_CAPACITY];
static size_t demoExpiryLength = 0;
static uint8_t wifiChannel = 1;

static volatile uint32_t activeTcpGeneration = 0;
static uint32_t nextTcpGeneration = 1;
static volatile uint32_t tcpQueueFaultGeneration = 0;

static volatile uint32_t busRxCount = 0;
static volatile uint32_t tabletRxCount = 0;
static volatile uint32_t tcpTxFrameCount = 0;
static volatile uint32_t tcpQueueDropCount = 0;
static volatile uint32_t stackQueueDropCount = 0;
static volatile uint32_t busTxFailCount = 0;
static uint32_t tcpWriteFailureCount = 0;

// ---------------------------------------------------------------------------
// Protocol encoding

static uint8_t xorChecksum(const uint8_t *data, size_t length) {
  uint8_t result = 0;
  for (size_t i = 0; i < length; ++i) result ^= data[i];
  return result;
}

static size_t encodeViewT(uint32_t canId, const uint8_t *data, uint8_t dlc,
                          uint8_t *out, size_t capacity) {
  if ((canId & ~CAN_ID_MASK) != 0 || dlc > 8 || capacity < (size_t)dlc + 9) return 0;
  out[0] = SOF;
  out[1] = (uint8_t)(5 + dlc);
  out[2] = EXTENDED_TYPE;
  out[3] = (uint8_t)(canId >> 24);
  out[4] = (uint8_t)(canId >> 16);
  out[5] = (uint8_t)(canId >> 8);
  out[6] = (uint8_t)canId;
  memcpy(out + 7, data, dlc);
  out[7 + dlc] = xorChecksum(out, 7 + dlc);
  out[8 + dlc] = EOF_BYTE;
  return (size_t)dlc + 9;
}

// Called from the CAN RX task and from the stack's write path (loop task).
static void queueForTablet(uint32_t canId, const uint8_t *data, uint8_t dlc) {
  const uint32_t generation = activeTcpGeneration;
  if (generation == 0 || tcpQueue == nullptr) return;
  QueuedTcpFrame frame;
  const size_t length = encodeViewT(canId, data, dlc > 8 ? 8 : dlc, frame.data,
                                    sizeof(frame.data));
  if (!length) return;
  frame.connectionGeneration = generation;
  frame.length = (uint8_t)length;
  if (xQueueSend(tcpQueue, &frame, 0) != pdTRUE) {
    ++tcpQueueDropCount;
    tcpQueueFaultGeneration = generation;
  }
}

static void queueForStack(const isobus::CANMessageFrame &frame) {
  if (xQueueSend(stackRxQueue, &frame, 0) != pdTRUE) ++stackQueueDropCount;
}

// ---------------------------------------------------------------------------
// CAN side

// The only twai_receive() caller: fans each bus frame out to the stack and
// the tablet, independent of how busy the Arduino loop is.
static void canRxTask(void *) {
  twai_message_t message;
  for (;;) {
    const esp_err_t error = twai_receive(&message, pdMS_TO_TICKS(100));
    if (error == ESP_ERR_TIMEOUT) continue;
    if (error != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(10)); // driver stopped/uninstalled
      continue;
    }
    if (message.rtr || message.data_length_code > 8) continue;
    ++busRxCount;

    isobus::CANMessageFrame frame = {};
    frame.identifier = message.identifier;
    frame.isExtendedFrame = message.extd;
    frame.dataLength = message.data_length_code;
    memcpy(frame.data, message.data, frame.dataLength);
    queueForStack(frame);
    if (message.extd) queueForTablet(message.identifier, message.data, message.data_length_code);
  }
}

static bool busTxErrorPassive() {
  twai_status_info_t status;
  return twai_get_status_info(&status) == ESP_OK &&
         (status.state != TWAI_STATE_RUNNING ||
          status.tx_error_counter >= TWAI_ERROR_PASSIVE_LIMIT);
}

void BridgedTWAIPlugin::open() {
  isobus::TWAIPlugin::open();
  if (tcpQueue == nullptr) tcpQueue = xQueueCreate(TCP_QUEUE_LENGTH, sizeof(QueuedTcpFrame));
  if (stackRxQueue == nullptr)
    stackRxQueue = xQueueCreate(STACK_RX_QUEUE_LENGTH, sizeof(isobus::CANMessageFrame));
  if (tcpQueue == nullptr || stackRxQueue == nullptr) {
    Serial.println("[BRIDGE] FATAL: queue allocation failed");
    return;
  }
  if (!rxTaskStarted &&
      xTaskCreatePinnedToCore(canRxTask, "can-rx", 4096, nullptr, 5, nullptr, 0) == pdPASS) {
    rxTaskStarted = true;
  }
}

// Frames are already buffered by canRxTask; never block the loop here.
bool BridgedTWAIPlugin::read_frame(isobus::CANMessageFrame &canFrame) {
  return stackRxQueue != nullptr && xQueueReceive(stackRxQueue, &canFrame, 0) == pdTRUE;
}

bool BridgedTWAIPlugin::write_frame(const isobus::CANMessageFrame &canFrame) {
  twai_message_t message = {};
  message.identifier = canFrame.identifier;
  message.extd = canFrame.isExtendedFrame;
  message.data_length_code = canFrame.dataLength;
  memcpy(message.data, canFrame.data, canFrame.dataLength);

  if (twai_transmit(&message, 0) != ESP_OK) {
    // Without another node to ACK, the controller sits error-passive retrying
    // one frame forever. If the tablet is the real receiver, give it the frame
    // anyway; otherwise it stays queued in the stack and is retried on the
    // next update(), as with TWAIPlugin's zero timeout.
    if (activeTcpGeneration == 0 || !busTxErrorPassive()) return false;
    ++busTxFailCount;
  }
  if (canFrame.isExtendedFrame)
    queueForTablet(canFrame.identifier, canFrame.data, canFrame.dataLength);
  return true;
}

bool has_pending_stack_frames() {
  return stackRxQueue != nullptr && uxQueueMessagesWaiting(stackRxQueue) > 0;
}

// ---------------------------------------------------------------------------
// TCP side (only ever touched by tcpTask)

static void closeTcpClient(const char *reason) {
  if (activeTcpGeneration == 0 && !tcpClient) return;
  activeTcpGeneration = 0;
  tcpClient.stop();
  tcpBufferLength = 0;
  Serial.printf("[BRIDGE] Tablet disconnected: %s\n", reason);
}

static bool sendToTcp(const uint8_t *data, size_t length) {
  if (activeTcpGeneration == 0 || !tcpClient.connected()) return false;

  const int socketFd = tcpClient.fd();
  const uint32_t startedAt = millis();
  size_t sent = 0;
  while (sent < length && tcpClient.connected()) {
    const int result = lwip_send(socketFd, data + sent, length - sent, MSG_DONTWAIT);
    if (result > 0) {
      sent += (size_t)result;
      continue;
    }
    if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) break;
    if ((uint32_t)(millis() - startedAt) >= TCP_WRITE_TIMEOUT_MS) break;
    vTaskDelay(1);
  }

  if (sent == length) return true;
  ++tcpWriteFailureCount;
  closeTcpClient("write failed or timed out");
  return false;
}

static void sendQueuedFrames() {
  if (activeTcpGeneration == 0) return;
  if (tcpQueueFaultGeneration == activeTcpGeneration) {
    // The app has no way to detect a lost frame, so a gap must end the session.
    closeTcpClient("CAN-to-TCP queue overflow");
    return;
  }

  uint8_t batch[TCP_CAN_BATCH_CAPACITY];
  size_t batchLength = 0;
  uint32_t batchFrames = 0;
  QueuedTcpFrame frame;
  const uint32_t generation = activeTcpGeneration;

  while (xQueuePeek(tcpQueue, &frame, 0) == pdTRUE) {
    if (frame.connectionGeneration != generation) {
      xQueueReceive(tcpQueue, &frame, 0); // stale, from a previous connection
      continue;
    }
    if (batchLength + frame.length > sizeof(batch)) break;
    if (xQueueReceive(tcpQueue, &frame, 0) != pdTRUE) break;
    memcpy(batch + batchLength, frame.data, frame.length);
    batchLength += frame.length;
    ++batchFrames;
  }

  if (batchLength && sendToTcp(batch, batchLength)) tcpTxFrameCount += batchFrames;
}

static bool sendFrame(uint8_t type, const uint8_t *header, size_t headerLength,
                      const uint8_t *data, size_t dataLength) {
  if (2 + headerLength + dataLength > 255) return false;
  uint8_t frame[260];
  size_t pos = 0;
  frame[pos++] = SOF;
  frame[pos++] = (uint8_t)(1 + headerLength + dataLength);
  frame[pos++] = type;
  if (headerLength) memcpy(frame + pos, header, headerLength);
  pos += headerLength;
  if (dataLength) memcpy(frame + pos, data, dataLength);
  pos += dataLength;
  frame[pos] = xorChecksum(frame, pos);
  frame[pos + 1] = EOF_BYTE;
  return sendToTcp(frame, pos + 2);
}

static void loadObjectDictionary() {
  licenseKeyLength = min(preferences.getBytesLength("license"), OD_VALUE_CAPACITY);
  if (licenseKeyLength) preferences.getBytes("license", licenseKey, licenseKeyLength);
  demoExpiryLength = min(preferences.getBytesLength("demoExpiry"), OD_VALUE_CAPACITY);
  if (demoExpiryLength) preferences.getBytes("demoExpiry", demoExpiry, demoExpiryLength);
  wifiChannel = preferences.getUChar("wifiChannel", 1);
  if (wifiChannel < 1 || wifiChannel > 13) wifiChannel = 1;
}

static void storeObjectDictionaryValue(const char *key, uint8_t *destination,
                                       size_t &destinationLength,
                                       const uint8_t *data, size_t length) {
  destinationLength = min(length, OD_VALUE_CAPACITY);
  if (destinationLength) {
    memcpy(destination, data, destinationLength);
    preferences.putBytes(key, destination, destinationLength);
  } else {
    preferences.remove(key);
  }
}

static void handleObjectDictionaryRead(uint16_t index, uint8_t subIndex) {
  const uint8_t *data = nullptr;
  size_t length = 0;

  if (index == OD_DEVICE && subIndex == OD_DEVICE_ID) {
    data = reinterpret_cast<const uint8_t *>(deviceId);
    length = strlen(deviceId);
  } else if (index == OD_LICENSE && subIndex == OD_LICENSE_KEY) {
    data = licenseKey;
    length = licenseKeyLength;
  } else if (index == OD_LICENSE && subIndex == OD_DEMO_EXPIRY) {
    data = demoExpiry;
    length = demoExpiryLength;
  } else if (index == OD_SETTINGS && subIndex == OD_WIFI_CHANNEL) {
    data = &wifiChannel;
    length = 1;
  } else {
    Serial.printf("[BRIDGE] OD read unsupported: %04X:%02X\n", index, subIndex);
  }

  const uint8_t header[] = {OD_READ_RESPONSE, (uint8_t)index, (uint8_t)(index >> 8), subIndex};
  sendFrame(OBJECT_DICTIONARY_TYPE, header, sizeof(header), data, length);
  Serial.printf("[BRIDGE] OD read: %04X:%02X (%u bytes)\n", index, subIndex, (unsigned)length);
}

static void handleObjectDictionaryWrite(uint16_t index, uint8_t subIndex,
                                        const uint8_t *data, size_t length) {
  if (index == OD_LICENSE && subIndex == OD_LICENSE_KEY) {
    storeObjectDictionaryValue("license", licenseKey, licenseKeyLength, data, length);
  } else if (index == OD_LICENSE && subIndex == OD_DEMO_EXPIRY) {
    storeObjectDictionaryValue("demoExpiry", demoExpiry, demoExpiryLength, data, length);
  } else if (index == OD_SETTINGS && subIndex == OD_WIFI_CHANNEL && length == 1 &&
             data[0] >= 1 && data[0] <= 13) {
    // Applied on the next boot, when the SoftAP is started (and ignored
    // while joined to a network, whose channel the SoftAP has to share).
    wifiChannel = data[0];
    preferences.putUChar("wifiChannel", wifiChannel);
  } else {
    Serial.printf("[BRIDGE] OD write unsupported: %04X:%02X\n", index, subIndex);
    return;
  }
  Serial.printf("[BRIDGE] OD write: %04X:%02X (%u bytes)\n", index, subIndex, (unsigned)length);
}

static void handleObjectDictionaryFrame(const uint8_t *frame, uint8_t bodyLength) {
  // Body: A5, command, index LSB, index MSB, sub-index, optional data.
  if (bodyLength < 5) return;
  const uint8_t command = frame[3];
  const uint16_t index = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  const uint8_t subIndex = frame[6];
  const uint8_t *data = frame + 7;
  const size_t dataLength = bodyLength - 5;

  if (command == OD_READ && dataLength == 0) {
    handleObjectDictionaryRead(index, subIndex);
  } else if (command == OD_WRITE) {
    handleObjectDictionaryWrite(index, subIndex, data, dataLength);
  } else {
    Serial.printf("[BRIDGE] Unknown OD command: %02X\n", command);
  }
}

static void handleCanControllerCommand(uint8_t command, const uint8_t *data,
                                       size_t dataLength) {
  const uint8_t baud = CAN_BAUD_250K;
  if (command == COMMAND_GET_CAN_BAUD_RATE && dataLength == 0) {
    sendFrame(command, nullptr, 0, &baud, 1);
  } else if (command == COMMAND_SET_CAN_BAUD_RATE && dataLength == 1) {
    if (data[0] == CAN_BAUD_250K) {
      sendFrame(command, nullptr, 0, &baud, 1);
    } else {
      sendFrame(COMMAND_DEVICE_BUSY, nullptr, 0, nullptr, 0);
      Serial.printf("[BRIDGE] Refused CAN baud rate code %02X (ISOBUS is fixed at 250k)\n", data[0]);
    }
  }
}

// A CAN frame from the tablet goes to the bus and, because TWAI never
// receives its own transmissions, straight into our own stack as well.
static void handleTabletCanFrame(uint32_t canId, const uint8_t *data, uint8_t dlc) {
  ++tabletRxCount;
  isobus::CANMessageFrame frame = {};
  frame.identifier = canId;
  frame.isExtendedFrame = true;
  frame.dataLength = dlc;
  memcpy(frame.data, data, dlc);
  queueForStack(frame);

  twai_message_t message = {};
  message.identifier = canId;
  message.extd = 1;
  message.data_length_code = dlc;
  memcpy(message.data, data, dlc);
  const TickType_t timeout = busTxErrorPassive() ? 0 : TABLET_TO_BUS_TIMEOUT;
  if (twai_transmit(&message, timeout) != ESP_OK) ++busTxFailCount;
}

static bool processOneTcpFrame() {
  while (tcpBufferLength && tcpBuffer[0] != SOF) {
    memmove(tcpBuffer, tcpBuffer + 1, --tcpBufferLength);
  }
  if (tcpBufferLength < 2) return false;
  const uint8_t bodyLength = tcpBuffer[1];
  if (bodyLength < 1) {
    memmove(tcpBuffer, tcpBuffer + 1, --tcpBufferLength);
    return true;
  }
  const size_t totalLength = bodyLength + 4;
  if (tcpBufferLength < totalLength) return false;
  if (tcpBuffer[totalLength - 1] != EOF_BYTE ||
      xorChecksum(tcpBuffer, totalLength - 2) != tcpBuffer[totalLength - 2]) {
    memmove(tcpBuffer, tcpBuffer + 1, --tcpBufferLength);
    return true;
  }

  const uint8_t type = tcpBuffer[2];
  if (type == OBJECT_DICTIONARY_TYPE) {
    handleObjectDictionaryFrame(tcpBuffer, bodyLength);
  } else if (type == COMMAND_GET_CAN_BAUD_RATE || type == COMMAND_SET_CAN_BAUD_RATE) {
    handleCanControllerCommand(type, tcpBuffer + 3, bodyLength - 1);
  } else if (type == EXTENDED_TYPE && bodyLength >= 5 && bodyLength <= 13) {
    const uint32_t canId = ((uint32_t)tcpBuffer[3] << 24) | ((uint32_t)tcpBuffer[4] << 16) |
                           ((uint32_t)tcpBuffer[5] << 8) | tcpBuffer[6];
    if ((canId & ~CAN_ID_MASK) == 0) handleTabletCanFrame(canId, tcpBuffer + 7, bodyLength - 5);
  }

  memmove(tcpBuffer, tcpBuffer + totalLength, tcpBufferLength - totalLength);
  tcpBufferLength -= totalLength;
  return true;
}

static void handleTcpInput() {
  if (activeTcpGeneration == 0) return;
  while (tcpClient.available() > 0 && tcpBufferLength < sizeof(tcpBuffer)) {
    const int count = tcpClient.read(tcpBuffer + tcpBufferLength, sizeof(tcpBuffer) - tcpBufferLength);
    if (count <= 0) break;
    tcpBufferLength += (size_t)count;
  }
  while (processOneTcpFrame()) {}
  if (!tcpClient.connected()) closeTcpClient("peer disconnected");
}

static void acceptTcpClient() {
  if (activeTcpGeneration != 0 && tcpClient.connected()) return;
  if (activeTcpGeneration != 0) closeTcpClient("peer disconnected");
  WiFiClient candidate = tcpServer.available();
  if (!candidate) return;
  if (tcpClient) tcpClient.stop();
  tcpClient = candidate;
  tcpClient.setNoDelay(true);
  tcpBufferLength = 0;
  xQueueReset(tcpQueue);
  uint32_t generation = nextTcpGeneration++;
  if (generation == 0) generation = nextTcpGeneration++;
  tcpQueueFaultGeneration = 0;
  activeTcpGeneration = generation;
  Serial.printf("[BRIDGE] Tablet connected: %s\n", tcpClient.remoteIP().toString().c_str());
}

static void tcpTask(void *) {
  for (;;) {
    acceptTcpClient();
    handleTcpInput();
    sendQueuedFrames();
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------

WifiSettings wifi_settings() {
  preferences.begin("viewt-od", false);
  loadObjectDictionary();
  return {HOSTNAME, AP_SSID, AP_PASSWORD, AP_IP, wifiChannel};
}

void begin() {
  if (tcpQueue == nullptr || stackRxQueue == nullptr) {
    Serial.println("[BRIDGE] CAN plugin not opened; bridge disabled");
    return;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(deviceId, sizeof(deviceId), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  if (xTaskCreatePinnedToCore(tcpTask, "viewt-tcp", 6144, nullptr, 2, nullptr, 0) != pdPASS) {
    Serial.println("[BRIDGE] TCP task creation failed; bridge disabled");
    return;
  }
  MDNS.addService("viewt", "tcp", TCP_PORT);
  MDNS.addServiceTxt("viewt", "tcp", "id", String(deviceId));
  Serial.printf("[BRIDGE] TCP port %u (%s or %s.local), device ID %s\n", TCP_PORT,
                AP_IP.toString().c_str(), HOSTNAME, deviceId);
}

void print_status() {
  Serial.printf("[BRIDGE] tablet=%d bus_rx=%lu tablet_rx=%lu tcp_tx=%lu tcp_queued=%u "
                "tcp_drop=%lu stack_drop=%lu bus_tx_fail=%lu tcp_fail=%lu\n",
                activeTcpGeneration != 0, (unsigned long)busRxCount,
                (unsigned long)tabletRxCount, (unsigned long)tcpTxFrameCount,
                tcpQueue ? (unsigned)uxQueueMessagesWaiting(tcpQueue) : 0,
                (unsigned long)tcpQueueDropCount, (unsigned long)stackQueueDropCount,
                (unsigned long)busTxFailCount, (unsigned long)tcpWriteFailureCount);
}

} // namespace viewt_bridge

#endif

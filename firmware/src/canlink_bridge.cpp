/*
 * Wi-Fi/CAN bridge (Proemion CANlink Byte Command Protocol) - see
 * canlink_bridge.h.
 *
 * Ported from the standalone M5Stack Core2 bridge (ISOBUS_DEMO/
 * proeminon_main.cpp). Differences: the TWAI controller is shared with the
 * ISOBUS stack through BridgedTWAIPlugin, the CAN baud rate is fixed at
 * ISOBUS's 250 kbit/s, and the device ID is this board's own MAC.
 */
#if defined(ENABLE_CANLINK_BRIDGE)

#include "canlink_bridge.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include <errno.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>

#include "driver/twai.h"
#include "esp_mac.h"

#include "debug_log.h"

namespace canlink_bridge {

// SoftAP defaults (SSID and password can be changed on the /wifi page).
static constexpr char AP_SSID[] = "AgIsoCanLink";
static constexpr char AP_PASSWORD[] = "12345678";
static const IPAddress AP_IP(192, 168, 0, 34);
static constexpr char HOSTNAME[] = "joystick-auxn";
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

// Object dictionary entries (device ID, license storage, Wi-Fi channel) that
// clients read and write.
static constexpr uint16_t OD_DEVICE = 0x3007;
static constexpr uint8_t OD_DEVICE_ID = 0x02;
static constexpr uint16_t OD_LICENSE = 0x3009;
static constexpr uint8_t OD_LICENSE_KEY = 0x0C;
static constexpr uint8_t OD_DEMO_EXPIRY = 0x0D;
static constexpr uint16_t OD_SETTINGS = 0x3000;
static constexpr uint8_t OD_WIFI_CHANNEL = 0x0F;
static constexpr size_t OD_VALUE_CAPACITY = 64;

// No PSRAM on this build, so queues are sized for internal RAM.
static constexpr UBaseType_t TCP_QUEUE_LENGTH = 512;
static constexpr UBaseType_t STACK_RX_QUEUE_LENGTH = 256;
// Bytes handed to the socket but not yet accepted by it. Only whole frames go
// in, so the stream never breaks mid-frame.
static constexpr size_t TCP_TX_BUFFER_SIZE = 2048;
// A client that stops reading (zero TCP window) keeps its session: ISOBUS
// recovers from lost frames by itself (VT status timeout, TP timeouts), while
// a closed connection is fatal for a client that doesn't reconnect.
// Meanwhile the CAN-to-TCP queue is compacted to the newest frame per CAN ID
// whenever it runs nearly full, and once more after a stall this long, so
// the latest of every periodic message (our Working Set Maintenance, AUX-N
// input status, ...) reaches the client as soon as it reads again. Throwing
// the whole backlog away instead made AgIsoVT drop the working set after
// stalls well under its 3 s maintenance timeout.
static constexpr uint32_t TCP_STALL_LOG_MS = 500;
static constexpr uint32_t TCP_STALE_BACKLOG_MS = 2000;
static constexpr UBaseType_t TCP_QUEUE_COMPACT_SPACES = 64; // compact below this much room
static constexpr uint32_t TCP_QUEUE_COMPACT_INTERVAL_MS = 50;
static constexpr size_t STALL_HISTORY_LENGTH = 5;
// Dead peer (e.g. phone left the AP without closing) detected after
// idle + interval * count seconds. (Keepalive only probes an idle link; a
// stalled one is left to the stall handling above.)
static constexpr int TCP_KEEPALIVE_IDLE_S = 5;
static constexpr int TCP_KEEPALIVE_INTERVAL_S = 2;
static constexpr int TCP_KEEPALIVE_COUNT = 3;
// Tablet frames are sent from the TCP task, which can afford to wait briefly
// for room in the TWAI TX queue.
static constexpr TickType_t TABLET_TO_BUS_TIMEOUT = pdMS_TO_TICKS(20);
static constexpr uint32_t TWAI_ERROR_PASSIVE_LIMIT = 128;

struct QueuedTcpFrame {
  uint32_t connectionGeneration;
  uint8_t length;
  uint8_t data[17];
};

struct StallRecord {
  uint32_t startMs;
  uint32_t durationMs;        // 0 while ongoing
  int8_t rssiStart;           // station RSSI in dBm, 0 = not joined to a network
  int8_t rssiEnd;
  uint32_t queueDrops;        // frames that didn't fit the queue
  uint32_t duplicatesDropped; // older frames superseded by a newer one of the same CAN ID
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

static volatile uint32_t busRxCount = 0;
static volatile uint32_t tabletRxCount = 0;
static volatile uint32_t tcpTxFrameCount = 0;
static volatile uint32_t tcpQueueDropCount = 0;
static volatile uint32_t stackQueueDropCount = 0;
static volatile uint32_t busTxFailCount = 0;
static uint32_t tcpWriteErrorCount = 0;

// Outgoing bytes (tcpTask only): txBuffer[txSent..txLength) is still unsent.
static uint8_t txBuffer[TCP_TX_BUFFER_SIZE];
static size_t txLength = 0;
static size_t txSent = 0;

// Client stalls (tcpTask only)
static uint32_t stallStartMs = 0; // 0 = not stalled
static uint32_t stallStartQueueDrops = 0;
static uint32_t stallStartDuplicates = 0;
static int8_t stallStartRssi = 0;
static bool stallLogged = false;
static bool compactBacklog = false;
static uint32_t lastCompactMs = 0;
static uint32_t stallCount = 0;
static uint32_t longestStallMs = 0;
static uint32_t duplicateFramesDropped = 0;
static StallRecord stallHistory[STALL_HISTORY_LENGTH]; // ring, newest at stallCount - 1

// Scratch space for compactTcpQueue()
static QueuedTcpFrame compactFrames[TCP_QUEUE_LENGTH];
static uint32_t compactIds[TCP_QUEUE_LENGTH];

// Session bookkeeping for /debug (tcpTask only)
static String tabletAddress;
static uint32_t sessionStartMs = 0;
static uint32_t sessionStartTxFrames = 0;
static uint32_t sessionStartRxFrames = 0;
static uint32_t sessionCount = 0;
static String lastCloseReason;
static uint32_t lastCloseMs = 0;

// ---------------------------------------------------------------------------
// Protocol encoding

static uint8_t xorChecksum(const uint8_t *data, size_t length) {
  uint8_t result = 0;
  for (size_t i = 0; i < length; ++i) result ^= data[i];
  return result;
}

static size_t encodeCanFrame(uint32_t canId, const uint8_t *data, uint8_t dlc,
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
  const size_t length = encodeCanFrame(canId, data, dlc > 8 ? 8 : dlc, frame.data,
                                    sizeof(frame.data));
  if (!length) return;
  frame.connectionGeneration = generation;
  frame.length = (uint8_t)length;
  if (xQueueSend(tcpQueue, &frame, 0) != pdTRUE) ++tcpQueueDropCount;
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
    debug_log("[BRIDGE] FATAL: queue allocation failed");
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

static void closeTcpClient(const String &reason) {
  if (activeTcpGeneration == 0 && !tcpClient) return;
  activeTcpGeneration = 0;
  tcpClient.stop();
  tcpBufferLength = 0;
  txLength = txSent = 0;
  stallStartMs = 0;
  stallLogged = compactBacklog = false;
  lastCloseReason = reason;
  lastCloseMs = millis();
  debug_log("[BRIDGE] Tablet %s disconnected after %lu s (%lu frames out, %lu in): %s",
            tabletAddress.c_str(), (unsigned long)((lastCloseMs - sessionStartMs) / 1000),
            (unsigned long)(tcpTxFrameCount - sessionStartTxFrames),
            (unsigned long)(tabletRxCount - sessionStartRxFrames), reason.c_str());
}

// Whole frames only: false (frame dropped) if it doesn't fit.
static bool appendToTx(const uint8_t *data, size_t length) {
  if (txSent == txLength) {
    txSent = txLength = 0;
  } else if (txLength + length > sizeof(txBuffer)) {
    memmove(txBuffer, txBuffer + txSent, txLength - txSent);
    txLength -= txSent;
    txSent = 0;
  }
  if (txLength + length > sizeof(txBuffer)) return false;
  memcpy(txBuffer + txLength, data, length);
  txLength += length;
  return true;
}

static int8_t stationRssi() {
  return WiFi.status() == WL_CONNECTED ? (int8_t)WiFi.RSSI() : 0;
}

static String rssiText(int8_t rssi) {
  return rssi != 0 ? String(rssi) + " dBm" : String("n/a");
}

static uint32_t queuedFrameId(const QueuedTcpFrame &frame) {
  return ((uint32_t)frame.data[3] << 24) | ((uint32_t)frame.data[4] << 16) |
         ((uint32_t)frame.data[5] << 8) | frame.data[6];
}

// Drops every queued frame that has a newer one with the same CAN ID behind
// it, keeping the rest in order. Frames queued meanwhile by the CAN RX task or
// the stack land behind the kept ones, so the order stays chronological.
static void compactTcpQueue(uint32_t generation) {
  lastCompactMs = millis();
  const UBaseType_t waiting = uxQueueMessagesWaiting(tcpQueue);
  size_t count = 0;
  for (UBaseType_t i = 0; i < waiting; ++i) {
    if (xQueueReceive(tcpQueue, &compactFrames[count], 0) != pdTRUE) break;
    if (compactFrames[count].connectionGeneration == generation) ++count;
  }

  // Newest first: a frame is kept unless a newer one with its ID was kept.
  size_t keptIds = 0;
  uint32_t duplicates = 0;
  for (size_t i = count; i-- > 0;) {
    const uint32_t id = queuedFrameId(compactFrames[i]);
    bool newerKept = false;
    for (size_t k = 0; k < keptIds && !newerKept; ++k) newerKept = compactIds[k] == id;
    if (newerKept) {
      compactFrames[i].length = 0; // dropped
      ++duplicates;
    } else {
      compactIds[keptIds++] = id;
    }
  }
  // Back to the front of the queue, newest first, so the oldest ends up first.
  for (size_t i = count; i-- > 0;) {
    if (compactFrames[i].length != 0 &&
        xQueueSendToFront(tcpQueue, &compactFrames[i], 0) != pdTRUE)
      ++tcpQueueDropCount;
  }
  duplicateFramesDropped += duplicates;
  if (duplicates)
    debug_log("[BRIDGE] Tablet backlog: kept the newest frame of %u CAN IDs, dropped %lu "
              "older ones", (unsigned)keptIds, (unsigned long)duplicates);
}

static void endStall() {
  const uint32_t stalledMs = millis() - stallStartMs;
  if (stalledMs > longestStallMs) longestStallMs = stalledMs;
  if (stalledMs >= TCP_STALE_BACKLOG_MS) compactBacklog = true;
  if (stallLogged) {
    stallLogged = false;
    const int8_t rssi = stationRssi();
    const uint32_t queueDrops = tcpQueueDropCount - stallStartQueueDrops;
    debug_log("[BRIDGE] Tablet reading again after %lu ms (%lu frames dropped meanwhile), "
              "RSSI %s",
              (unsigned long)stalledMs, (unsigned long)queueDrops, rssiText(rssi).c_str());
    StallRecord &record = stallHistory[(stallCount - 1) % STALL_HISTORY_LENGTH];
    record.durationMs = stalledMs;
    record.rssiEnd = rssi;
    record.queueDrops = queueDrops;
    record.duplicatesDropped = duplicateFramesDropped - stallStartDuplicates;
  }
  stallStartMs = 0;
}

// Sends as much of txBuffer as the socket takes, without blocking. False if
// the connection had to be closed.
static bool flushTx() {
  if (activeTcpGeneration == 0) return false;
  while (txSent < txLength) {
    const int result = lwip_send(tcpClient.fd(), txBuffer + txSent, txLength - txSent,
                                 MSG_DONTWAIT);
    if (result > 0) {
      txSent += (size_t)result;
      continue;
    }
    if (result == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      if (stallStartMs == 0) {
        stallStartMs = millis() | 1; // never 0 while stalled
        stallStartQueueDrops = tcpQueueDropCount;
        stallStartDuplicates = duplicateFramesDropped;
        stallStartRssi = stationRssi();
      }
      return true; // retried on the next round
    }
    const int error = errno;
    ++tcpWriteErrorCount;
    char reason[64];
    snprintf(reason, sizeof(reason), "write error %d (%s)", error, strerror(error));
    closeTcpClient(reason);
    return false;
  }
  if (stallStartMs != 0) endStall();
  return true;
}

static void sendQueuedFrames() {
  if (!flushTx()) return;
  const uint32_t generation = activeTcpGeneration;
  QueuedTcpFrame frame;

  if (txSent < txLength) {
    // Still stalled: leave the frames queued, compacted to the newest per CAN
    // ID once they nearly fill the queue; what still doesn't fit is dropped
    // by queueForTablet().
    const uint32_t now = millis();
    if (!stallLogged && now - stallStartMs >= TCP_STALL_LOG_MS) {
      stallLogged = true;
      stallHistory[stallCount % STALL_HISTORY_LENGTH] = {stallStartMs, 0, stallStartRssi, 0, 0, 0};
      ++stallCount;
      debug_log("[BRIDGE] Tablet %s stopped reading (RSSI %s at stall start, %s now, %u bytes "
                "unsent); keeping the session",
                tabletAddress.c_str(), rssiText(stallStartRssi).c_str(),
                rssiText(stationRssi()).c_str(), (unsigned)(txLength - txSent));
    }
    if (uxQueueSpacesAvailable(tcpQueue) < TCP_QUEUE_COMPACT_SPACES &&
        now - lastCompactMs >= TCP_QUEUE_COMPACT_INTERVAL_MS)
      compactTcpQueue(generation);
    return;
  }

  if (compactBacklog) {
    compactBacklog = false;
    compactTcpQueue(generation);
  }

  uint32_t frames = 0;
  while (xQueuePeek(tcpQueue, &frame, 0) == pdTRUE) {
    if (frame.connectionGeneration != generation) {
      xQueueReceive(tcpQueue, &frame, 0); // stale, from a previous connection
      continue;
    }
    if (!appendToTx(frame.data, frame.length)) break; // full: next round
    xQueueReceive(tcpQueue, &frame, 0);
    ++frames;
  }
  tcpTxFrameCount += frames;
  flushTx();
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
  if (activeTcpGeneration == 0 || !appendToTx(frame, pos + 2)) return false;
  return flushTx();
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
    debug_log("[BRIDGE] OD read unsupported: %04X:%02X", index, subIndex);
  }

  const uint8_t header[] = {OD_READ_RESPONSE, (uint8_t)index, (uint8_t)(index >> 8), subIndex};
  sendFrame(OBJECT_DICTIONARY_TYPE, header, sizeof(header), data, length);
  debug_log("[BRIDGE] OD read: %04X:%02X (%u bytes)", index, subIndex, (unsigned)length);
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
    debug_log("[BRIDGE] OD write unsupported: %04X:%02X", index, subIndex);
    return;
  }
  debug_log("[BRIDGE] OD write: %04X:%02X (%u bytes)", index, subIndex, (unsigned)length);
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
    debug_log("[BRIDGE] Unknown OD command: %02X", command);
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
      debug_log("[BRIDGE] Refused CAN baud rate code %02X (ISOBUS is fixed at 250k)", data[0]);
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

  if (activeTcpGeneration == 0) return false; // a reply failed and closed the session
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

static void enableKeepalive(int socketFd) {
  const int on = 1;
  setsockopt(socketFd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPIDLE, &TCP_KEEPALIVE_IDLE_S, sizeof(int));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPINTVL, &TCP_KEEPALIVE_INTERVAL_S, sizeof(int));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPCNT, &TCP_KEEPALIVE_COUNT, sizeof(int));
}

// A VT client only talks to a VT whose address claim it has seen, and the
// VT only knows nodes whose claims reached it. Claims made before this
// tablet connected (the joystick's at boot, a VT that was already running)
// never crossed the bridge, so ask everyone to claim again: the tablet side,
// the bus and our own stack. A global Request for Address Claim must be
// answered by every control function; it's sent from the NULL address since
// the bridge has none of its own. Repeated shortly after the connection
// opens, in case the client isn't processing frames yet.
static constexpr uint32_t CLAIM_REQUEST_DELAYS_MS[] = {0, 1000, 3000};
static size_t claimRequestsSent = 0;

static void requestAddressClaims() {
  constexpr uint32_t REQUEST_PGN = 0xEA00;
  constexpr uint32_t ADDRESS_CLAIM_PGN = 0xEE00;
  constexpr uint8_t GLOBAL_ADDRESS = 0xFF;
  constexpr uint8_t NULL_ADDRESS = 0xFE;
  constexpr uint32_t PRIORITY = 6;
  // PDU1: the destination is the PGN's low byte
  const uint32_t canId = (PRIORITY << 26) | ((REQUEST_PGN | GLOBAL_ADDRESS) << 8) | NULL_ADDRESS;
  const uint8_t data[3] = {(uint8_t)ADDRESS_CLAIM_PGN, (uint8_t)(ADDRESS_CLAIM_PGN >> 8),
                           (uint8_t)(ADDRESS_CLAIM_PGN >> 16)};

  queueForTablet(canId, data, sizeof(data));

  isobus::CANMessageFrame frame = {};
  frame.identifier = canId;
  frame.isExtendedFrame = true;
  frame.dataLength = sizeof(data);
  memcpy(frame.data, data, sizeof(data));
  queueForStack(frame);

  twai_message_t message = {};
  message.identifier = canId;
  message.extd = 1;
  message.data_length_code = sizeof(data);
  memcpy(message.data, data, sizeof(data));
  if (twai_transmit(&message, 0) != ESP_OK) ++busTxFailCount;
}

static void acceptTcpClient() {
  if (activeTcpGeneration != 0 && !tcpClient.connected()) closeTcpClient("peer disconnected");
  WiFiClient candidate = tcpServer.available();
  if (!candidate) return;
  // A new connection replaces the current one: after a Wi-Fi drop the app
  // reconnects while our old socket may still look alive, and it would
  // otherwise sit unserved until that socket times out.
  if (activeTcpGeneration != 0)
    closeTcpClient("replaced by new connection from " + candidate.remoteIP().toString());
  if (tcpClient) tcpClient.stop();
  tcpClient = candidate;
  tcpClient.setNoDelay(true);
  enableKeepalive(tcpClient.fd());
  tcpBufferLength = 0;
  xQueueReset(tcpQueue);
  uint32_t generation = nextTcpGeneration++;
  if (generation == 0) generation = nextTcpGeneration++;
  txLength = txSent = 0;
  tabletAddress = tcpClient.remoteIP().toString() + ":" + String(tcpClient.remotePort());
  sessionStartMs = millis();
  sessionStartTxFrames = tcpTxFrameCount;
  sessionStartRxFrames = tabletRxCount;
  ++sessionCount;
  activeTcpGeneration = generation;
  claimRequestsSent = 0;
  debug_log("[BRIDGE] Tablet connected: %s", tabletAddress.c_str());
}

static void sendScheduledClaimRequests() {
  constexpr size_t COUNT = sizeof(CLAIM_REQUEST_DELAYS_MS) / sizeof(CLAIM_REQUEST_DELAYS_MS[0]);
  if (activeTcpGeneration == 0 || claimRequestsSent >= COUNT ||
      millis() - sessionStartMs < CLAIM_REQUEST_DELAYS_MS[claimRequestsSent])
    return;
  requestAddressClaims();
  debug_log("[BRIDGE] Request for Address Claim %u/%u sent", (unsigned)++claimRequestsSent,
            (unsigned)COUNT);
}

static void tcpTask(void *) {
  for (;;) {
    acceptTcpClient();
    sendScheduledClaimRequests();
    handleTcpInput();
    sendQueuedFrames();
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------

WifiSettings wifi_settings() {
  preferences.begin("canlink-od", false);
  loadObjectDictionary();
  return {HOSTNAME, AP_SSID, AP_PASSWORD, AP_IP, wifiChannel};
}

void begin() {
  if (tcpQueue == nullptr || stackRxQueue == nullptr) {
    debug_log("[BRIDGE] CAN plugin not opened; bridge disabled");
    return;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(deviceId, sizeof(deviceId), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  if (xTaskCreatePinnedToCore(tcpTask, "canlink-tcp", 6144, nullptr, 2, nullptr, 0) != pdPASS) {
    debug_log("[BRIDGE] TCP task creation failed; bridge disabled");
    return;
  }
  wifi_add_mdns_service("canlink", "tcp", TCP_PORT, "id", deviceId);
  debug_log("[BRIDGE] TCP port %u (%s or %s.local), device ID %s", TCP_PORT,
            AP_IP.toString().c_str(), wifi_hostname(), deviceId);
}

void debug_status(String &out) {
  const uint32_t now = millis();
  out += "CANlink bridge (port " + String(TCP_PORT) + ", device ID " + String(deviceId) + ")\n";
  if (activeTcpGeneration != 0) {
    out += "  tablet " + tabletAddress + ", connected " +
           String((now - sessionStartMs) / 1000) + " s, " +
           String(tcpTxFrameCount - sessionStartTxFrames) + " frames out, " +
           String(tabletRxCount - sessionStartRxFrames) + " in\n";
  } else {
    out += "  no tablet connected\n";
  }
  out += "  sessions " + String(sessionCount);
  if (lastCloseMs != 0)
    out += ", last closed " + String((now - lastCloseMs) / 1000) + " s ago: " + lastCloseReason;
  out += "\n  frames: bus rx " + String(busRxCount) + ", tablet rx " + String(tabletRxCount) +
         ", tcp tx " + String(tcpTxFrameCount) + ", queued " +
         String(tcpQueue ? uxQueueMessagesWaiting(tcpQueue) : 0) + "\n";
  out += "  drops: tcp queue " + String(tcpQueueDropCount) + ", older duplicates " +
         String(duplicateFramesDropped) + ", stack queue " + String(stackQueueDropCount) +
         ", bus tx " + String(busTxFailCount) + ", tcp write errors " +
         String(tcpWriteErrorCount) + "\n";
  out += "  tablet stalls " + String(stallCount) + ", longest " + String(longestStallMs) +
         " ms";
  if (stallStartMs != 0) out += ", stalled now for " + String(now - stallStartMs) + " ms";
  out += "\n";
  // Last stalls, newest first (only those of at least TCP_STALL_LOG_MS)
  const uint32_t recorded = min(stallCount, (uint32_t)STALL_HISTORY_LENGTH);
  for (uint32_t i = 0; i < recorded; ++i) {
    const StallRecord &record = stallHistory[(stallCount - 1 - i) % STALL_HISTORY_LENGTH];
    char line[160];
    snprintf(line, sizeof(line), "    at %lu.%lu s: ", (unsigned long)(record.startMs / 1000),
             (unsigned long)(record.startMs % 1000 / 100));
    out += line;
    if (record.durationMs == 0) {
      out += "ongoing, RSSI " + rssiText(record.rssiStart) + " at start\n";
      continue;
    }
    snprintf(line, sizeof(line),
             "%lu ms, RSSI %s -> %s, %lu queue drops, %lu older duplicates dropped\n",
             (unsigned long)record.durationMs, rssiText(record.rssiStart).c_str(),
             rssiText(record.rssiEnd).c_str(), (unsigned long)record.queueDrops,
             (unsigned long)record.duplicatesDropped);
    out += line;
  }
}

void print_status() {
  Serial.printf("[BRIDGE] tablet=%d bus_rx=%lu tablet_rx=%lu tcp_tx=%lu tcp_queued=%u "
                "tcp_drop=%lu stack_drop=%lu bus_tx_fail=%lu tcp_err=%lu\n",
                activeTcpGeneration != 0, (unsigned long)busRxCount,
                (unsigned long)tabletRxCount, (unsigned long)tcpTxFrameCount,
                tcpQueue ? (unsigned)uxQueueMessagesWaiting(tcpQueue) : 0,
                (unsigned long)tcpQueueDropCount, (unsigned long)stackQueueDropCount,
                (unsigned long)busTxFailCount, (unsigned long)tcpWriteErrorCount);
}

} // namespace canlink_bridge

#endif

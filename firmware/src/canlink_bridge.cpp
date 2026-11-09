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
// One client per port, like the CANlink wireless 4000's per-connection
// listen ports (0x3010:0x18 ... 0x3016:0x18, 30000 ... 30006 by default).
// Only the first is announced over mDNS; give every further app on the same
// tablet its own port (30001, 30002, ...), so their sessions don't replace
// each other.
static constexpr uint16_t TCP_PORT = 30000;
static constexpr size_t CLIENT_COUNT = 4; // ports TCP_PORT ... TCP_PORT + 3

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
// 1: a client's frames also go to the other clients (CANlink MultiTalk);
// 0: only to the bus and our stack (MultiPoint). Default 1, since two apps on
// one tablet (VT + TC) only see each other this way: TWAI doesn't receive
// its own transmissions, so nothing comes back from the bus.
static constexpr uint16_t OD_MULTI = 0x3008;
static constexpr uint8_t OD_MULTITALK_ENABLE = 0x04;
static constexpr size_t OD_VALUE_CAPACITY = 64;

// No PSRAM on this build, so queues are sized for internal RAM. A client's
// queue is only created when its port is first used.
static constexpr UBaseType_t TCP_QUEUE_LENGTH = 512;
static constexpr UBaseType_t STACK_RX_QUEUE_LENGTH = 256;
// Bytes handed to the socket but not yet accepted by it. Only whole frames go
// in, so the stream never breaks mid-frame.
static constexpr size_t TCP_TX_BUFFER_SIZE = 2048;
static constexpr size_t TCP_RX_BUFFER_SIZE = 2048;
// A client that stops reading (zero TCP window) keeps its session: ISOBUS
// recovers from lost frames by itself (VT status timeout, TP timeouts), while
// a closed connection is fatal for a client that doesn't reconnect.
// Meanwhile its CAN-to-TCP queue is compacted to the newest frame per CAN ID
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
// Client frames are sent from the TCP task, which can afford to wait briefly
// for room in the TWAI TX queue.
static constexpr TickType_t CLIENT_TO_BUS_TIMEOUT = pdMS_TO_TICKS(20);
static constexpr uint32_t TWAI_ERROR_PASSIVE_LIMIT = 128;

// A VT client only talks to a VT whose address claim it has seen, and the
// VT only knows nodes whose claims reached it. Claims made before a client
// connected (the joystick's at boot, a VT or TC that was already running)
// never crossed the bridge, so ask everyone to claim again: all clients,
// the bus and our own stack. Repeated shortly after the connection opens,
// in case the client isn't processing frames yet.
static constexpr uint32_t CLAIM_REQUEST_DELAYS_MS[] = {0, 1000, 3000};
static constexpr size_t CLAIM_REQUEST_COUNT =
    sizeof(CLAIM_REQUEST_DELAYS_MS) / sizeof(CLAIM_REQUEST_DELAYS_MS[0]);

struct QueuedTcpFrame {
  uint32_t connectionGeneration;
  uint8_t length;
  uint8_t data[17];
};

struct StallRecord {
  uint32_t startMs;
  uint32_t durationMs;        // 0 while ongoing
  uint16_t port;
  int8_t rssiStart;           // station RSSI in dBm, 0 = not joined to a network
  int8_t rssiEnd;
  uint32_t queueDrops;        // frames that didn't fit the queue
  uint32_t duplicatesDropped; // older frames superseded by a newer one of the same CAN ID
};

// One TCP client, on its own port. Everything but generation, queue and
// queueDrops is only touched by tcpTask.
struct Client {
  uint16_t port = 0;
  WiFiServer *server = nullptr;
  WiFiClient socket;
  // Nonzero while connected. Frames queued under an older generation are
  // stale (from a previous connection on this port) and get skipped.
  volatile uint32_t generation = 0;
  QueueHandle_t queue = nullptr; // CAN frames -> this client
  volatile uint32_t queueDrops = 0;

  uint8_t rxBuffer[TCP_RX_BUFFER_SIZE];
  size_t rxLength = 0;
  // txBuffer[txSent..txLength) is still unsent.
  uint8_t txBuffer[TCP_TX_BUFFER_SIZE];
  size_t txLength = 0;
  size_t txSent = 0;

  // Stalls
  uint32_t stallStartMs = 0; // 0 = not stalled
  uint32_t stallStartQueueDrops = 0;
  uint32_t stallStartDuplicates = 0;
  int8_t stallStartRssi = 0;
  bool stallLogged = false;
  bool compactBacklog = false;
  uint32_t lastCompactMs = 0;
  uint32_t duplicatesDropped = 0;

  // Session bookkeeping for /debug
  String address;
  uint32_t sessionStartMs = 0;
  uint32_t framesOut = 0; // to the client
  uint32_t framesIn = 0;  // from the client
  size_t claimRequestsSent = 0;
};

static Client clients[CLIENT_COUNT];
static QueueHandle_t stackRxQueue = nullptr; // bus + client frames -> stack
static bool rxTaskStarted = false;
static uint32_t nextTcpGeneration = 1;

static Preferences preferences;
static char deviceId[18];
static uint8_t licenseKey[OD_VALUE_CAPACITY];
static size_t licenseKeyLength = 0;
static uint8_t demoExpiry[OD_VALUE_CAPACITY];
static size_t demoExpiryLength = 0;
static uint8_t wifiChannel = 1;
static volatile uint8_t multiTalk = 1;

static volatile uint32_t busRxCount = 0;
static volatile uint32_t clientRxCount = 0;
static volatile uint32_t tcpTxFrameCount = 0;
static volatile uint32_t tcpQueueDropCount = 0;
static volatile uint32_t stackQueueDropCount = 0;
static volatile uint32_t busTxFailCount = 0;
static uint32_t tcpWriteErrorCount = 0;

// Client stalls, all ports (tcpTask only)
static uint32_t stallCount = 0;
static uint32_t longestStallMs = 0;
static uint32_t duplicateFramesDropped = 0;
static StallRecord stallHistory[STALL_HISTORY_LENGTH]; // ring, newest at stallCount - 1

// Scratch space for compactQueue() (tcpTask only, one client at a time)
static QueuedTcpFrame compactFrames[TCP_QUEUE_LENGTH];
static uint32_t compactIds[TCP_QUEUE_LENGTH];

// Sessions, all ports (tcpTask only)
static uint32_t sessionCount = 0;
static String lastCloseReason;
static uint32_t lastCloseMs = 0;

static bool anyClientConnected() {
  for (const Client &client : clients)
    if (client.generation != 0) return true;
  return false;
}

static size_t connectedClientCount() {
  size_t count = 0;
  for (const Client &client : clients)
    if (client.generation != 0) ++count;
  return count;
}

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

// Queues a frame for every connected client except `skip` (the one it came
// from, if any). Called from the CAN RX task, the stack's write path (loop
// task) and tcpTask.
static void queueForClients(uint32_t canId, const uint8_t *data, uint8_t dlc,
                            const Client *skip = nullptr) {
  QueuedTcpFrame frame;
  const size_t length = encodeCanFrame(canId, data, dlc > 8 ? 8 : dlc, frame.data,
                                    sizeof(frame.data));
  if (!length) return;
  frame.length = (uint8_t)length;
  for (Client &client : clients) {
    if (&client == skip) continue;
    const uint32_t generation = client.generation;
    if (generation == 0 || client.queue == nullptr) continue;
    frame.connectionGeneration = generation;
    if (xQueueSend(client.queue, &frame, 0) != pdTRUE) {
      ++tcpQueueDropCount;
      ++client.queueDrops;
    }
  }
}

static void queueForStack(const isobus::CANMessageFrame &frame) {
  if (xQueueSend(stackRxQueue, &frame, 0) != pdTRUE) ++stackQueueDropCount;
}

// ---------------------------------------------------------------------------
// CAN side

// The only twai_receive() caller: fans each bus frame out to the stack and
// the clients, independent of how busy the Arduino loop is.
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
    if (message.extd) queueForClients(message.identifier, message.data, message.data_length_code);
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
  if (stackRxQueue == nullptr)
    stackRxQueue = xQueueCreate(STACK_RX_QUEUE_LENGTH, sizeof(isobus::CANMessageFrame));
  if (stackRxQueue == nullptr) {
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
    // one frame forever. If a client is the real receiver, give it the frame
    // anyway; otherwise it stays queued in the stack and is retried on the
    // next update(), as with TWAIPlugin's zero timeout.
    if (!anyClientConnected() || !busTxErrorPassive()) return false;
    ++busTxFailCount;
  }
  if (canFrame.isExtendedFrame)
    queueForClients(canFrame.identifier, canFrame.data, canFrame.dataLength);
  return true;
}

bool has_pending_stack_frames() {
  return stackRxQueue != nullptr && uxQueueMessagesWaiting(stackRxQueue) > 0;
}

// ---------------------------------------------------------------------------
// TCP side (only ever touched by tcpTask)

static void closeClient(Client &client, const String &reason) {
  if (client.generation == 0 && !client.socket) return;
  const bool wasConnected = client.generation != 0;
  client.generation = 0;
  client.socket.stop();
  client.rxLength = 0;
  client.txLength = client.txSent = 0;
  client.stallStartMs = 0;
  client.stallLogged = client.compactBacklog = false;
  if (!wasConnected) return;
  lastCloseReason = String(client.port) + ": " + reason;
  lastCloseMs = millis();
  debug_log("[BRIDGE] Client %s on port %u disconnected after %lu s (%lu frames out, %lu in): %s",
            client.address.c_str(), (unsigned)client.port,
            (unsigned long)((lastCloseMs - client.sessionStartMs) / 1000),
            (unsigned long)client.framesOut, (unsigned long)client.framesIn, reason.c_str());
}

// Whole frames only: false (frame dropped) if it doesn't fit.
static bool appendToTx(Client &client, const uint8_t *data, size_t length) {
  if (client.txSent == client.txLength) {
    client.txSent = client.txLength = 0;
  } else if (client.txLength + length > sizeof(client.txBuffer)) {
    memmove(client.txBuffer, client.txBuffer + client.txSent, client.txLength - client.txSent);
    client.txLength -= client.txSent;
    client.txSent = 0;
  }
  if (client.txLength + length > sizeof(client.txBuffer)) return false;
  memcpy(client.txBuffer + client.txLength, data, length);
  client.txLength += length;
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
static void compactQueue(Client &client) {
  client.lastCompactMs = millis();
  const uint32_t generation = client.generation;
  const UBaseType_t waiting = uxQueueMessagesWaiting(client.queue);
  size_t count = 0;
  for (UBaseType_t i = 0; i < waiting; ++i) {
    if (xQueueReceive(client.queue, &compactFrames[count], 0) != pdTRUE) break;
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
        xQueueSendToFront(client.queue, &compactFrames[i], 0) != pdTRUE) {
      ++tcpQueueDropCount;
      ++client.queueDrops;
    }
  }
  duplicateFramesDropped += duplicates;
  client.duplicatesDropped += duplicates;
  if (duplicates)
    debug_log("[BRIDGE] Port %u backlog: kept the newest frame of %u CAN IDs, dropped %lu "
              "older ones", (unsigned)client.port, (unsigned)keptIds, (unsigned long)duplicates);
}

static void endStall(Client &client) {
  const uint32_t stalledMs = millis() - client.stallStartMs;
  if (stalledMs > longestStallMs) longestStallMs = stalledMs;
  if (stalledMs >= TCP_STALE_BACKLOG_MS) client.compactBacklog = true;
  if (client.stallLogged) {
    client.stallLogged = false;
    const int8_t rssi = stationRssi();
    const uint32_t queueDrops = client.queueDrops - client.stallStartQueueDrops;
    debug_log("[BRIDGE] Port %u client reading again after %lu ms (%lu frames dropped "
              "meanwhile), RSSI %s",
              (unsigned)client.port, (unsigned long)stalledMs, (unsigned long)queueDrops,
              rssiText(rssi).c_str());
    StallRecord &record = stallHistory[(stallCount - 1) % STALL_HISTORY_LENGTH];
    if (record.port == client.port && record.durationMs == 0) {
      record.durationMs = stalledMs;
      record.rssiEnd = rssi;
      record.queueDrops = queueDrops;
      record.duplicatesDropped = client.duplicatesDropped - client.stallStartDuplicates;
    }
  }
  client.stallStartMs = 0;
}

// Sends as much of the client's txBuffer as the socket takes, without
// blocking. False if the connection had to be closed.
static bool flushTx(Client &client) {
  if (client.generation == 0) return false;
  while (client.txSent < client.txLength) {
    const int result = lwip_send(client.socket.fd(), client.txBuffer + client.txSent,
                                 client.txLength - client.txSent, MSG_DONTWAIT);
    if (result > 0) {
      client.txSent += (size_t)result;
      continue;
    }
    if (result == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      if (client.stallStartMs == 0) {
        client.stallStartMs = millis() | 1; // never 0 while stalled
        client.stallStartQueueDrops = client.queueDrops;
        client.stallStartDuplicates = client.duplicatesDropped;
        client.stallStartRssi = stationRssi();
      }
      return true; // retried on the next round
    }
    const int error = errno;
    ++tcpWriteErrorCount;
    char reason[64];
    snprintf(reason, sizeof(reason), "write error %d (%s)", error, strerror(error));
    closeClient(client, reason);
    return false;
  }
  if (client.stallStartMs != 0) endStall(client);
  return true;
}

static void sendQueuedFrames(Client &client) {
  if (!flushTx(client)) return;
  const uint32_t generation = client.generation;
  QueuedTcpFrame frame;

  if (client.txSent < client.txLength) {
    // Still stalled: leave the frames queued, compacted to the newest per CAN
    // ID once they nearly fill the queue; what still doesn't fit is dropped
    // by queueForClients().
    const uint32_t now = millis();
    if (!client.stallLogged && now - client.stallStartMs >= TCP_STALL_LOG_MS) {
      client.stallLogged = true;
      stallHistory[stallCount % STALL_HISTORY_LENGTH] = {
          client.stallStartMs, 0, client.port, client.stallStartRssi, 0, 0, 0};
      ++stallCount;
      debug_log("[BRIDGE] Client %s on port %u stopped reading (RSSI %s at stall start, %s now, "
                "%u bytes unsent); keeping the session",
                client.address.c_str(), (unsigned)client.port,
                rssiText(client.stallStartRssi).c_str(), rssiText(stationRssi()).c_str(),
                (unsigned)(client.txLength - client.txSent));
    }
    if (uxQueueSpacesAvailable(client.queue) < TCP_QUEUE_COMPACT_SPACES &&
        now - client.lastCompactMs >= TCP_QUEUE_COMPACT_INTERVAL_MS)
      compactQueue(client);
    return;
  }

  if (client.compactBacklog) {
    client.compactBacklog = false;
    compactQueue(client);
  }

  uint32_t frames = 0;
  while (xQueuePeek(client.queue, &frame, 0) == pdTRUE) {
    if (frame.connectionGeneration != generation) {
      xQueueReceive(client.queue, &frame, 0); // stale, from a previous connection
      continue;
    }
    if (!appendToTx(client, frame.data, frame.length)) break; // full: next round
    xQueueReceive(client.queue, &frame, 0);
    ++frames;
  }
  tcpTxFrameCount += frames;
  client.framesOut += frames;
  flushTx(client);
}

static bool sendFrame(Client &client, uint8_t type, const uint8_t *header, size_t headerLength,
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
  if (client.generation == 0 || !appendToTx(client, frame, pos + 2)) return false;
  return flushTx(client);
}

static void loadObjectDictionary() {
  licenseKeyLength = min(preferences.getBytesLength("license"), OD_VALUE_CAPACITY);
  if (licenseKeyLength) preferences.getBytes("license", licenseKey, licenseKeyLength);
  demoExpiryLength = min(preferences.getBytesLength("demoExpiry"), OD_VALUE_CAPACITY);
  if (demoExpiryLength) preferences.getBytes("demoExpiry", demoExpiry, demoExpiryLength);
  wifiChannel = preferences.getUChar("wifiChannel", 1);
  if (wifiChannel < 1 || wifiChannel > 13) wifiChannel = 1;
  multiTalk = preferences.getUChar("multiTalk", 1) ? 1 : 0;
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

static void handleObjectDictionaryRead(Client &client, uint16_t index, uint8_t subIndex) {
  const uint8_t *data = nullptr;
  size_t length = 0;
  const uint8_t multiTalkValue = multiTalk;

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
  } else if (index == OD_MULTI && subIndex == OD_MULTITALK_ENABLE) {
    data = &multiTalkValue;
    length = 1;
  } else {
    debug_log("[BRIDGE] OD read unsupported: %04X:%02X", index, subIndex);
  }

  const uint8_t header[] = {OD_READ_RESPONSE, (uint8_t)index, (uint8_t)(index >> 8), subIndex};
  sendFrame(client, OBJECT_DICTIONARY_TYPE, header, sizeof(header), data, length);
  debug_log("[BRIDGE] OD read on port %u: %04X:%02X (%u bytes)", (unsigned)client.port, index,
            subIndex, (unsigned)length);
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
  } else if (index == OD_MULTI && subIndex == OD_MULTITALK_ENABLE && length == 1 &&
             data[0] <= 1) {
    multiTalk = data[0]; // applies right away
    preferences.putUChar("multiTalk", data[0]);
  } else {
    debug_log("[BRIDGE] OD write unsupported: %04X:%02X", index, subIndex);
    return;
  }
  debug_log("[BRIDGE] OD write: %04X:%02X (%u bytes)", index, subIndex, (unsigned)length);
}

static void handleObjectDictionaryFrame(Client &client, const uint8_t *frame,
                                        uint8_t bodyLength) {
  // Body: A5, command, index LSB, index MSB, sub-index, optional data.
  if (bodyLength < 5) return;
  const uint8_t command = frame[3];
  const uint16_t index = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  const uint8_t subIndex = frame[6];
  const uint8_t *data = frame + 7;
  const size_t dataLength = bodyLength - 5;

  if (command == OD_READ && dataLength == 0) {
    handleObjectDictionaryRead(client, index, subIndex);
  } else if (command == OD_WRITE) {
    handleObjectDictionaryWrite(index, subIndex, data, dataLength);
  } else {
    debug_log("[BRIDGE] Unknown OD command: %02X", command);
  }
}

static void handleCanControllerCommand(Client &client, uint8_t command, const uint8_t *data,
                                       size_t dataLength) {
  const uint8_t baud = CAN_BAUD_250K;
  if (command == COMMAND_GET_CAN_BAUD_RATE && dataLength == 0) {
    sendFrame(client, command, nullptr, 0, &baud, 1);
  } else if (command == COMMAND_SET_CAN_BAUD_RATE && dataLength == 1) {
    if (data[0] == CAN_BAUD_250K) {
      sendFrame(client, command, nullptr, 0, &baud, 1);
    } else {
      sendFrame(client, COMMAND_DEVICE_BUSY, nullptr, 0, nullptr, 0);
      debug_log("[BRIDGE] Refused CAN baud rate code %02X (ISOBUS is fixed at 250k)", data[0]);
    }
  }
}

// A CAN frame from a client goes to the bus and, because TWAI never receives
// its own transmissions, straight into our own stack and (MultiTalk) the
// other clients as well.
static void handleClientCanFrame(Client &client, uint32_t canId, const uint8_t *data,
                                 uint8_t dlc) {
  ++clientRxCount;
  ++client.framesIn;
  isobus::CANMessageFrame frame = {};
  frame.identifier = canId;
  frame.isExtendedFrame = true;
  frame.dataLength = dlc;
  memcpy(frame.data, data, dlc);
  queueForStack(frame);
  if (multiTalk) queueForClients(canId, data, dlc, &client);

  twai_message_t message = {};
  message.identifier = canId;
  message.extd = 1;
  message.data_length_code = dlc;
  memcpy(message.data, data, dlc);
  const TickType_t timeout = busTxErrorPassive() ? 0 : CLIENT_TO_BUS_TIMEOUT;
  if (twai_transmit(&message, timeout) != ESP_OK) ++busTxFailCount;
}

static bool processOneTcpFrame(Client &client) {
  uint8_t *buffer = client.rxBuffer;
  size_t &length = client.rxLength;
  while (length && buffer[0] != SOF) {
    memmove(buffer, buffer + 1, --length);
  }
  if (length < 2) return false;
  const uint8_t bodyLength = buffer[1];
  if (bodyLength < 1) {
    memmove(buffer, buffer + 1, --length);
    return true;
  }
  const size_t totalLength = bodyLength + 4;
  if (length < totalLength) return false;
  if (buffer[totalLength - 1] != EOF_BYTE ||
      xorChecksum(buffer, totalLength - 2) != buffer[totalLength - 2]) {
    memmove(buffer, buffer + 1, --length);
    return true;
  }

  const uint8_t type = buffer[2];
  if (type == OBJECT_DICTIONARY_TYPE) {
    handleObjectDictionaryFrame(client, buffer, bodyLength);
  } else if (type == COMMAND_GET_CAN_BAUD_RATE || type == COMMAND_SET_CAN_BAUD_RATE) {
    handleCanControllerCommand(client, type, buffer + 3, bodyLength - 1);
  } else if (type == EXTENDED_TYPE && bodyLength >= 5 && bodyLength <= 13) {
    const uint32_t canId = ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[4] << 16) |
                           ((uint32_t)buffer[5] << 8) | buffer[6];
    if ((canId & ~CAN_ID_MASK) == 0) handleClientCanFrame(client, canId, buffer + 7, bodyLength - 5);
  }

  if (client.generation == 0) return false; // a reply failed and closed the session
  memmove(buffer, buffer + totalLength, length - totalLength);
  length -= totalLength;
  return true;
}

static void handleTcpInput(Client &client) {
  if (client.generation == 0) return;
  while (client.socket.available() > 0 && client.rxLength < sizeof(client.rxBuffer)) {
    const int count = client.socket.read(client.rxBuffer + client.rxLength,
                                         sizeof(client.rxBuffer) - client.rxLength);
    if (count <= 0) break;
    client.rxLength += (size_t)count;
  }
  while (processOneTcpFrame(client)) {}
  if (client.generation != 0 && !client.socket.connected()) closeClient(client, "peer disconnected");
}

static void enableKeepalive(int socketFd) {
  const int on = 1;
  setsockopt(socketFd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPIDLE, &TCP_KEEPALIVE_IDLE_S, sizeof(int));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPINTVL, &TCP_KEEPALIVE_INTERVAL_S, sizeof(int));
  setsockopt(socketFd, IPPROTO_TCP, TCP_KEEPCNT, &TCP_KEEPALIVE_COUNT, sizeof(int));
}

// A global Request for Address Claim must be answered by every control
// function; it's sent from the NULL address since the bridge has none of its
// own.
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

  queueForClients(canId, data, sizeof(data));

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

static void acceptClient(Client &client) {
  if (client.generation != 0 && !client.socket.connected())
    closeClient(client, "peer disconnected");
  if (client.server == nullptr) return;
  WiFiClient candidate = client.server->available();
  if (!candidate) return;
  if (client.queue == nullptr) {
    client.queue = xQueueCreate(TCP_QUEUE_LENGTH, sizeof(QueuedTcpFrame));
    if (client.queue == nullptr) {
      debug_log("[BRIDGE] No memory for a port %u queue; refused %s", (unsigned)client.port,
                candidate.remoteIP().toString().c_str());
      candidate.stop();
      return;
    }
  }
  // A new connection replaces the current one on the same port: after a
  // Wi-Fi drop the app reconnects while our old socket may still look alive,
  // and it would otherwise sit unserved until that socket times out.
  if (client.generation != 0)
    closeClient(client, "replaced by new connection from " + candidate.remoteIP().toString());
  if (client.socket) client.socket.stop();
  client.socket = candidate;
  client.socket.setNoDelay(true);
  enableKeepalive(client.socket.fd());
  client.rxLength = 0;
  client.txLength = client.txSent = 0;
  xQueueReset(client.queue);
  uint32_t generation = nextTcpGeneration++;
  if (generation == 0) generation = nextTcpGeneration++;
  client.address = client.socket.remoteIP().toString() + ":" + String(client.socket.remotePort());
  client.sessionStartMs = millis();
  client.framesOut = client.framesIn = 0;
  client.claimRequestsSent = 0;
  ++sessionCount;
  client.generation = generation;
  debug_log("[BRIDGE] Client connected on port %u: %s (%u of %u ports in use)",
            (unsigned)client.port, client.address.c_str(), (unsigned)connectedClientCount(),
            (unsigned)CLIENT_COUNT);
}

static void sendScheduledClaimRequests(Client &client) {
  if (client.generation == 0 || client.claimRequestsSent >= CLAIM_REQUEST_COUNT ||
      millis() - client.sessionStartMs < CLAIM_REQUEST_DELAYS_MS[client.claimRequestsSent])
    return;
  requestAddressClaims();
  debug_log("[BRIDGE] Request for Address Claim %u/%u sent (port %u connected)",
            (unsigned)++client.claimRequestsSent, (unsigned)CLAIM_REQUEST_COUNT,
            (unsigned)client.port);
}

static void tcpTask(void *) {
  for (;;) {
    for (Client &client : clients) {
      acceptClient(client);
      sendScheduledClaimRequests(client);
      handleTcpInput(client);
      if (client.generation != 0) sendQueuedFrames(client);
    }
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
  if (stackRxQueue == nullptr) {
    debug_log("[BRIDGE] CAN plugin not opened; bridge disabled");
    return;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(deviceId, sizeof(deviceId), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  for (size_t i = 0; i < CLIENT_COUNT; ++i) {
    Client &client = clients[i];
    client.port = (uint16_t)(TCP_PORT + i);
    client.server = new WiFiServer(client.port);
    client.server->begin();
    client.server->setNoDelay(true);
  }

  if (xTaskCreatePinnedToCore(tcpTask, "canlink-tcp", 6144, nullptr, 2, nullptr, 0) != pdPASS) {
    debug_log("[BRIDGE] TCP task creation failed; bridge disabled");
    return;
  }
  wifi_add_mdns_service("canlink", "tcp", TCP_PORT, "id", deviceId);
  debug_log("[BRIDGE] TCP ports %u-%u (%s or %s.local), device ID %s, MultiTalk %s",
            (unsigned)TCP_PORT, (unsigned)(TCP_PORT + CLIENT_COUNT - 1),
            AP_IP.toString().c_str(), wifi_hostname(), deviceId, multiTalk ? "on" : "off");
}

void debug_status(String &out) {
  const uint32_t now = millis();
  out += "CANlink bridge (ports " + String(TCP_PORT) + "-" +
         String(TCP_PORT + CLIENT_COUNT - 1) + ", device ID " + String(deviceId) +
         ", MultiTalk " + (multiTalk ? "on" : "off") + ")\n";
  for (const Client &client : clients) {
    if (client.generation == 0) continue;
    out += "  port " + String(client.port) + ": " + client.address + ", connected " +
           String((now - client.sessionStartMs) / 1000) + " s, " + String(client.framesOut) +
           " frames out, " + String(client.framesIn) + " in, queued " +
           String(client.queue ? uxQueueMessagesWaiting(client.queue) : 0);
    if (client.stallStartMs != 0)
      out += ", stalled now for " + String(now - client.stallStartMs) + " ms";
    out += "\n";
  }
  if (!anyClientConnected()) out += "  no client connected\n";
  out += "  sessions " + String(sessionCount);
  if (lastCloseMs != 0)
    out += ", last closed " + String((now - lastCloseMs) / 1000) + " s ago (port " +
           lastCloseReason + ")";
  out += "\n  frames: bus rx " + String(busRxCount) + ", client rx " + String(clientRxCount) +
         ", tcp tx " + String(tcpTxFrameCount) + "\n";
  out += "  drops: tcp queues " + String(tcpQueueDropCount) + ", older duplicates " +
         String(duplicateFramesDropped) + ", stack queue " + String(stackQueueDropCount) +
         ", bus tx " + String(busTxFailCount) + ", tcp write errors " +
         String(tcpWriteErrorCount) + "\n";
  out += "  client stalls " + String(stallCount) + ", longest " + String(longestStallMs) +
         " ms\n";
  // Last stalls, newest first (only those of at least TCP_STALL_LOG_MS)
  const uint32_t recorded = min(stallCount, (uint32_t)STALL_HISTORY_LENGTH);
  for (uint32_t i = 0; i < recorded; ++i) {
    const StallRecord &record = stallHistory[(stallCount - 1 - i) % STALL_HISTORY_LENGTH];
    char line[160];
    snprintf(line, sizeof(line), "    port %u at %lu.%lu s: ", (unsigned)record.port,
             (unsigned long)(record.startMs / 1000), (unsigned long)(record.startMs % 1000 / 100));
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
  Serial.printf("[BRIDGE] clients=%u bus_rx=%lu client_rx=%lu tcp_tx=%lu tcp_drop=%lu "
                "stack_drop=%lu bus_tx_fail=%lu tcp_err=%lu\n",
                (unsigned)connectedClientCount(), (unsigned long)busRxCount,
                (unsigned long)clientRxCount, (unsigned long)tcpTxFrameCount,
                (unsigned long)tcpQueueDropCount, (unsigned long)stackQueueDropCount,
                (unsigned long)busTxFailCount, (unsigned long)tcpWriteErrorCount);
}

} // namespace canlink_bridge

#endif

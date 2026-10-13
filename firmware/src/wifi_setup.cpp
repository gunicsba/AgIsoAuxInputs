#if defined(ENABLE_OTA_WEBSERVER)

#include "wifi_setup.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include "debug_log.h"

static const IPAddress AP_NETMASK(255, 255, 255, 0);
// A connection attempt scans every channel and the SoftAP follows the radio,
// knocking phones off the AP for a few seconds. So a network that can't be
// joined is retried with a backoff rather than continuously (Arduino's
// auto-reconnect).
static constexpr uint32_t STA_RETRY_MIN_MS = 30000;
static constexpr uint32_t STA_RETRY_MAX_MS = 300000;
// A new AP password is applied after the page response has gone out, since
// restarting the AP drops the browser that asked for it.
static constexpr uint32_t AP_RESTART_DELAY_MS = 1000;

static WebServer *server = nullptr;
static WifiSettings settings;
static Preferences prefs;
static String staSsid;
static String staPassword;
static String apPassword;
static uint32_t apRestartAt = 0; // 0 = none pending
static uint32_t lastStaAttempt = 0;
static uint32_t staRetryMs = STA_RETRY_MIN_MS;

// Written from the Wi-Fi event task, read by the web server.
static volatile uint32_t staDisconnectCount = 0;
static volatile uint8_t lastStaDisconnectReason = 0;
static volatile uint32_t lastStaDisconnectMs = 0;
static volatile uint32_t staConnectedSinceMs = 0;
static volatile uint32_t apJoinCount = 0;
static volatile uint32_t apLeaveCount = 0;

static void startStation() {
  lastStaAttempt = millis();
  if (staSsid.isEmpty())
    return;
  WiFi.disconnect();
  WiFi.begin(staSsid.c_str(), staPassword.c_str());
  debug_log("[WIFI] Connecting to '%s'", staSsid.c_str());
}

// The AP's /24 and the station's network overlap: lwIP can't tell which
// interface a reply belongs to, so one side stops working.
static bool apSubnetClash() {
  if (!WiFi.isConnected())
    return false;
  const uint32_t mask = (uint32_t)WiFi.subnetMask() & (uint32_t)AP_NETMASK;
  return ((uint32_t)WiFi.localIP() & mask) == ((uint32_t)settings.apIp & mask);
}

static String htmlEscape(const String &text) {
  String out;
  out.reserve(text.length());
  for (char c : text) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c;
    }
  }
  return out;
}

static String macToString(const uint8_t *mac) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
  return text;
}

static void startAccessPoint() {
  // Joined to a network, the AP has to share that network's channel.
  const uint8_t channel = WiFi.isConnected() ? WiFi.channel() : settings.apChannel;
  if (!WiFi.softAPConfig(settings.apIp, settings.apIp, AP_NETMASK))
    debug_log("[WIFI] softAPConfig failed");
  if (!WiFi.softAP(settings.apSsid, apPassword.c_str(), channel, false, 4))
    debug_log("[WIFI] SoftAP setup failed");
  debug_log("[WIFI] AP '%s' ch %u at %s", settings.apSsid, channel,
            settings.apIp.toString().c_str());
}

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      staConnectedSinceMs = millis();
      debug_log("[WIFI] Joined '%s': %s ch %u %d dBm, http://%s.local/", staSsid.c_str(),
                WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.RSSI(),
                settings.hostname);
      if (apSubnetClash())
        debug_log("[WIFI] WARNING: network overlaps the access point's subnet");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      staConnectedSinceMs = 0;
      ++staDisconnectCount;
      lastStaDisconnectReason = info.wifi_sta_disconnected.reason;
      lastStaDisconnectMs = millis();
      debug_log("[WIFI] Station disconnected: reason %u (%s)", info.wifi_sta_disconnected.reason,
                WiFi.disconnectReasonName((wifi_err_reason_t)info.wifi_sta_disconnected.reason));
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      ++apJoinCount;
      debug_log("[WIFI] AP client joined: %s (%u on AP)",
                macToString(info.wifi_ap_staconnected.mac).c_str(), WiFi.softAPgetStationNum());
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      ++apLeaveCount;
      debug_log("[WIFI] AP client left: %s (%u on AP)",
                macToString(info.wifi_ap_stadisconnected.mac).c_str(), WiFi.softAPgetStationNum());
      break;
    default:
      break;
  }
}

static String stationStatus() {
  if (staSsid.isEmpty())
    return "not configured";
  const String name = "'" + htmlEscape(staSsid) + "'";
  switch (WiFi.status()) {
    case WL_CONNECTED:
      return "connected to " + name + ", IP <b>" + WiFi.localIP().toString() +
             "</b>, " + String(WiFi.RSSI()) + " dBm";
    case WL_NO_SSID_AVAIL:
      return name + " not found, next try in " +
             String((staRetryMs - (millis() - lastStaAttempt)) / 1000) + " s";
    case WL_CONNECT_FAILED:
      return "could not join " + name + " (wrong password?)";
    default:
      return "connecting to " + name + "...";
  }
}

static void redirectToWifiPage() {
  server->sendHeader("Location", "/wifi");
  server->send(303);
}

static void handleWifiPage() {
  const int scanCount = WiFi.scanComplete();

  String html = "<html><head><meta name='viewport' content='width=device-width'>";
  if (scanCount == WIFI_SCAN_RUNNING)
    html += "<meta http-equiv='refresh' content='3'>";
  html += "</head><body style='font-family:sans-serif'><h3>Wi-Fi</h3>";

  html += "<h4>Access point (always on)</h4>"
          "<p>SSID: <b>" + htmlEscape(settings.apSsid) + "</b> at " + settings.apIp.toString() +
          "<br>Password: <b>" + htmlEscape(apPassword) + "</b></p>"
          "<form method='post' action='/wifi/ap'>"
          "<p>New password (8-63 characters)<br>"
          "<input name='password' minlength='8' maxlength='63'></p>"
          "<p><button>Change password</button> (reconnect with the new one)</p></form>";

  html += "<h4>Network</h4><p>" + stationStatus() + "</p>";
  html += "<p>Name: <b>" + String(settings.hostname) + ".local</b></p>";
  if (apSubnetClash()) {
    html += "<p style='color:#b00'>This network overlaps the access point's " +
            settings.apIp.toString() + "/24 - one of them will not work. "
            "Use a network with a different subnet.</p>";
  }

  html += "<form method='post' action='/wifi/save'>"
          "<p>SSID<br><input id='ssid' name='ssid' value='" + htmlEscape(staSsid) + "'></p>"
          "<p>Password<br><input id='pw' name='password' type='password' "
          "placeholder='" + String(staSsid.isEmpty() ? "" : "unchanged if empty") + "'></p>"
          "<p><button>Connect</button></p></form>";

  // Scans only on request: the radio leaves the AP's channel while it scans,
  // which drops TCP sessions to the AP (the View-T app).
  if (scanCount == WIFI_SCAN_RUNNING) {
    html += "<p>Scanning...</p>";
  } else {
    html += "<form method='post' action='/wifi/scan'><button>Scan for networks</button> "
            "(interrupts the access point for a few seconds)</form>";
    if (scanCount > 0)
      html += "<ul>";
    for (int i = 0; i < scanCount; i++) {
      const String ssid = WiFi.SSID(i);
      if (ssid.isEmpty())
        continue;
      html += "<li><a href='#' data-ssid='" + htmlEscape(ssid) + "'>" + htmlEscape(ssid) +
              "</a> " + String(WiFi.RSSI(i)) + " dBm ch " + String(WiFi.channel(i)) +
              (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " (open)" : "") + "</li>";
    }
    if (scanCount > 0)
      html += "</ul>";
    html += "<script>document.querySelectorAll('[data-ssid]').forEach(a=>a.onclick=e=>{"
            "e.preventDefault();ssid.value=a.dataset.ssid;pw.value='';pw.focus();});</script>";
  }

  if (!staSsid.isEmpty()) {
    html += "<form method='post' action='/wifi/forget'><button>Forget network</button></form>";
  }
  html += "<p><a href='/'>Back</a></p></body></html>";
  server->send(200, "text/html", html);
}

static void handleWifiScan() {
  if (WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
    WiFi.scanDelete();
    WiFi.scanNetworks(true);
    debug_log("[WIFI] Scan started");
  }
  redirectToWifiPage();
}

static void handleWifiSave() {
  String ssid = server->arg("ssid");
  ssid.trim();
  const String password = server->arg("password");
  if (!ssid.isEmpty()) {
    // An empty password keeps the saved one for the same network.
    if (ssid != staSsid || !password.isEmpty())
      staPassword = password;
    staSsid = ssid;
    prefs.putString("ssid", staSsid);
    prefs.putString("password", staPassword);
    staRetryMs = STA_RETRY_MIN_MS;
    startStation();
  }
  redirectToWifiPage();
}

static void handleApPassword() {
  const String password = server->arg("password");
  if (password.length() >= 8 && password.length() <= 63) { // WPA2-PSK limits
    apPassword = password;
    prefs.putString("ap_password", apPassword);
    apRestartAt = millis() + AP_RESTART_DELAY_MS;
    if (apRestartAt == 0)
      apRestartAt = 1;
    debug_log("[WIFI] AP password changed");
  }
  redirectToWifiPage();
}

static void handleWifiForget() {
  prefs.remove("ssid");
  prefs.remove("password");
  staSsid = "";
  staPassword = "";
  WiFi.disconnect();
  debug_log("[WIFI] Network forgotten");
  redirectToWifiPage();
}

void wifi_setup(WebServer &webServer, const WifiSettings &wifiSettings) {
  server = &webServer;
  settings = wifiSettings;

  prefs.begin("wifi", false);
  staSsid = prefs.getString("ssid", "");
  staPassword = prefs.getString("password", "");
  apPassword = prefs.getString("ap_password", settings.apPassword);

  WiFi.onEvent(onWifiEvent);
  WiFi.persistent(false);       // our own NVS namespace holds the credentials
  WiFi.setHostname(settings.hostname); // before the STA interface starts
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);         // modem sleep delays TCP to the AP's clients
  WiFi.setAutoReconnect(false); // see STA_RETRY_MIN_MS
  startAccessPoint();
  startStation();

  if (MDNS.begin(settings.hostname)) {
    MDNS.addService("http", "tcp", 80);
  } else {
    debug_log("[WIFI] mDNS start failed");
  }

  server->on("/wifi", HTTP_GET, handleWifiPage);
  server->on("/wifi/scan", HTTP_POST, handleWifiScan);
  server->on("/wifi/save", HTTP_POST, handleWifiSave);
  server->on("/wifi/forget", HTTP_POST, handleWifiForget);
  server->on("/wifi/ap", HTTP_POST, handleApPassword);
}

void wifi_update() {
  if (apRestartAt != 0 && (int32_t)(millis() - apRestartAt) >= 0) {
    apRestartAt = 0;
    startAccessPoint();
  }

  if (WiFi.isConnected()) {
    staRetryMs = STA_RETRY_MIN_MS;
  } else if (!staSsid.isEmpty() && millis() - lastStaAttempt >= staRetryMs &&
             WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
    startStation();
    staRetryMs = min(staRetryMs * 2, STA_RETRY_MAX_MS);
  }
}

void wifi_debug_status(String &out) {
  const uint32_t now = millis();
  out += "Wi-Fi\n";
  out += "  AP '" + String(settings.apSsid) + "' " + settings.apIp.toString() + " ch " +
         String(WiFi.channel()) + ", " + String(WiFi.softAPgetStationNum()) +
         " client(s), joins " + String(apJoinCount) + " leaves " + String(apLeaveCount) + "\n";
  out += "  station: ";
  if (staSsid.isEmpty()) {
    out += "not configured\n";
  } else if (WiFi.isConnected()) {
    out += "'" + staSsid + "' " + WiFi.localIP().toString() + " " + String(WiFi.RSSI()) +
           " dBm, up " + String((now - staConnectedSinceMs) / 1000) + " s\n";
  } else {
    out += "'" + staSsid + "' not connected (status " + String((int)WiFi.status()) +
           "), next try in " + String((staRetryMs - (now - lastStaAttempt)) / 1000) + " s\n";
  }
  out += "  station disconnects " + String(staDisconnectCount);
  if (staDisconnectCount) {
    out += ", last: reason " + String(lastStaDisconnectReason) + " (" +
           WiFi.disconnectReasonName((wifi_err_reason_t)lastStaDisconnectReason) + ") " +
           String((now - lastStaDisconnectMs) / 1000) + " s ago";
  }
  out += "\n  hostname " + String(settings.hostname) + ".local, sleep " +
         String(WiFi.getSleep() ? "on" : "off") + "\n";
}

#endif

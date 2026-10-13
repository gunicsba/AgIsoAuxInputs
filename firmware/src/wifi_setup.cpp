#if defined(ENABLE_OTA_WEBSERVER)

#include "wifi_setup.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

static const IPAddress AP_NETMASK(255, 255, 255, 0);
// One connection attempt scans every channel and the SoftAP follows the
// radio, so a missing network is retried at this pace rather than
// continuously (Arduino's auto-reconnect), which would keep knocking phones
// off the AP.
static constexpr uint32_t STA_RETRY_MS = 30000;
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
static bool staWasConnected = false;
static uint32_t lastStaAttempt = 0;

static void startStation() {
  lastStaAttempt = millis();
  if (staSsid.isEmpty())
    return;
  WiFi.disconnect();
  WiFi.begin(staSsid.c_str(), staPassword.c_str());
  Serial.printf("[WIFI] Connecting to '%s'\n", staSsid.c_str());
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

static void startAccessPoint() {
  // Joined to a network, the AP has to share that network's channel.
  const uint8_t channel = WiFi.isConnected() ? WiFi.channel() : settings.apChannel;
  if (!WiFi.softAPConfig(settings.apIp, settings.apIp, AP_NETMASK))
    Serial.println("[WIFI] softAPConfig failed");
  if (!WiFi.softAP(settings.apSsid, apPassword.c_str(), channel, false, 4))
    Serial.println("[WIFI] SoftAP setup failed");
  Serial.printf("[WIFI] AP '%s' ch %u at %s\n", settings.apSsid, channel,
                settings.apIp.toString().c_str());
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
      return name + " not found, retrying every " + String(STA_RETRY_MS / 1000) + " s";
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
  int scanCount = WiFi.scanComplete();
  if (scanCount == WIFI_SCAN_FAILED) { // never scanned (or failed): start one
    WiFi.scanNetworks(true);
    scanCount = WIFI_SCAN_RUNNING;
  }

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

  if (scanCount == WIFI_SCAN_RUNNING) {
    html += "<p>Scanning...</p>";
  } else {
    html += "<p>Networks (<a href='/wifi/scan'>rescan</a>):</p><ul>";
    for (int i = 0; i < scanCount; i++) {
      const String ssid = WiFi.SSID(i);
      if (ssid.isEmpty())
        continue;
      html += "<li><a href='#' data-ssid='" + htmlEscape(ssid) + "'>" + htmlEscape(ssid) +
              "</a> " + String(WiFi.RSSI(i)) + " dBm" +
              (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " (open)" : "") + "</li>";
    }
    html += "</ul><script>document.querySelectorAll('[data-ssid]').forEach(a=>a.onclick=e=>{"
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
  }
  redirectToWifiPage();
}

static void handleWifiForget() {
  prefs.remove("ssid");
  prefs.remove("password");
  staSsid = "";
  staPassword = "";
  WiFi.disconnect();
  Serial.println("[WIFI] Network forgotten");
  redirectToWifiPage();
}

void wifi_setup(WebServer &webServer, const WifiSettings &wifiSettings) {
  server = &webServer;
  settings = wifiSettings;

  prefs.begin("wifi", false);
  staSsid = prefs.getString("ssid", "");
  staPassword = prefs.getString("password", "");
  apPassword = prefs.getString("ap_password", settings.apPassword);

  WiFi.persistent(false);       // our own NVS namespace holds the credentials
  WiFi.setHostname(settings.hostname); // before the STA interface starts
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false); // see STA_RETRY_MS
  startAccessPoint();
  startStation();

  if (MDNS.begin(settings.hostname)) {
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("[WIFI] mDNS start failed");
  }

  server->on("/wifi", HTTP_GET, handleWifiPage);
  server->on("/wifi/scan", HTTP_GET, handleWifiScan);
  server->on("/wifi/save", HTTP_POST, handleWifiSave);
  server->on("/wifi/forget", HTTP_POST, handleWifiForget);
  server->on("/wifi/ap", HTTP_POST, handleApPassword);
}

void wifi_update() {
  if (apRestartAt != 0 && (int32_t)(millis() - apRestartAt) >= 0) {
    apRestartAt = 0;
    startAccessPoint();
  }

  const bool connected = WiFi.isConnected();
  if (connected != staWasConnected) {
    staWasConnected = connected;
    if (connected) {
      Serial.printf("[WIFI] Connected to '%s': %s, http://%s.local/\n", staSsid.c_str(),
                    WiFi.localIP().toString().c_str(), settings.hostname);
      if (apSubnetClash())
        Serial.println("[WIFI] WARNING: network overlaps the access point's subnet");
    } else {
      Serial.println("[WIFI] Disconnected");
    }
  }
  if (!connected && !staSsid.isEmpty() && millis() - lastStaAttempt >= STA_RETRY_MS &&
      WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
    startStation();
  }
}

#endif

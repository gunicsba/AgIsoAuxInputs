/*
 * Wi-Fi: an always-on SoftAP plus, once configured on the /wifi page, a
 * station connection to an existing network.
 *
 * The SoftAP keeps the setup page (and the View-T bridge) reachable even when
 * the configured network is gone. On the station side the board announces
 * itself over mDNS as http://<hostname>.local, so a VT/PC on that network
 * doesn't need to know the DHCP address.
 *
 * In AP+STA mode the ESP32 has one radio: once the station joins a network,
 * the SoftAP moves to that network's channel.
 */
#pragma once

#if defined(ENABLE_OTA_WEBSERVER)

#include <Arduino.h>
#include <IPAddress.h>

class WebServer;

struct WifiSettings {
  const char *hostname; // mDNS / DHCP name, without ".local"
  const char *apSsid;
  const char *apPassword;
  IPAddress apIp;
  uint8_t apChannel;
};

// Starts the SoftAP, joins the saved network (if any), starts mDNS (with an
// _http._tcp service for the web page) and adds the /wifi pages to server.
void wifi_setup(WebServer &server, const WifiSettings &settings);

// Retries the configured network (with backoff) and applies a changed AP
// password.
void wifi_update();

// AP / station summary for the /debug page.
void wifi_debug_status(String &out);

#endif

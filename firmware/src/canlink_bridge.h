/*
 * Wi-Fi/CAN bridge speaking the Proemion CANlink Byte Command Protocol over
 * TCP ports 30000-30003, like a CANlink WiFi gateway.
 *
 * Any CANlink client (e.g. AgIsoVirtualTerminal with AgIsoStack's CANlink
 * plugin) can join the bus through it and act as the VT for this AUX-N
 * device without other CAN hardware.
 *
 * Each port takes one client, so several apps on one tablet (a VT and a TC,
 * say) share the bridge: each uses its own port, and a new connection only
 * replaces the previous one on the same port (an app reconnecting after a
 * Wi-Fi drop). Only port 30000 is announced over mDNS. Like the CANlink
 * wireless 4000's MultiTalk mode (object 0x3008:0x04, default on here), a
 * client's frames also go to the other clients; with it off (MultiPoint)
 * they only go to the bus and our stack.
 *
 * The ESP32-S3 has a single TWAI controller and AgIsoStack's TWAIPlugin
 * normally owns twai_receive(), so a second reader would steal frames from
 * the stack. Instead BridgedTWAIPlugin replaces TWAIPlugin and becomes the
 * only path to the controller:
 *
 *   bus RX          -> ISOBUS stack + all clients
 *   stack TX        -> bus + all clients
 *   client TX       -> bus + ISOBUS stack + other clients (TWAI never
 *                      receives its own transmissions, so the local loop
 *                      has to be explicit)
 *
 * With only tablet clients connected (nothing else on the bus), bus
 * transmits get no ACK. Frames are still delivered to the clients in that
 * case, so the whole
 * VT session (address claim, pool upload, aux assignment) works standalone.
 */
#pragma once

#if defined(ENABLE_CANLINK_BRIDGE)

#if !defined(ENABLE_OTA_WEBSERVER)
#error "The CANlink bridge needs ENABLE_OTA_WEBSERVER for its Wi-Fi setup"
#endif

#include "isobus/hardware_integration/twai_plugin.hpp"
#include "wifi_setup.h"

namespace canlink_bridge {

class BridgedTWAIPlugin : public isobus::TWAIPlugin {
public:
  using isobus::TWAIPlugin::TWAIPlugin;

  void open() override;
  bool read_frame(isobus::CANMessageFrame &canFrame) override;
  bool write_frame(const isobus::CANMessageFrame &canFrame) override;
};

// SoftAP "AgIsoCanLink" at 192.168.0.34; on a configured network the board
// is joystick-auxn.local (all three can be changed on the /wifi page).
WifiSettings wifi_settings();

// Starts the TCP servers (ports 30000-30003) and announces port 30000 over
// mDNS (_canlink._tcp). Call after wifi_setup() and CANHardwareInterface::start().
void begin();

// True while bus/tablet frames are waiting to be handed to the stack; the
// stack takes one frame per CANHardwareInterface::update().
bool has_pending_stack_frames();

void print_status();

// Session and counter summary for the /debug page.
void debug_status(String &out);

} // namespace canlink_bridge

#endif

/*
 * Wi-Fi/CAN bridge speaking the Proemion CANlink Byte Command Protocol over
 * TCP port 30000, like a CANlink WiFi gateway.
 *
 * Any CANlink client (e.g. AgIsoVirtualTerminal with AgIsoStack's CANlink
 * plugin) can join the bus through it and act as the VT for this AUX-N
 * device without other CAN hardware.
 *
 * The ESP32-S3 has a single TWAI controller and AgIsoStack's TWAIPlugin
 * normally owns twai_receive(), so a second reader would steal frames from
 * the stack. Instead BridgedTWAIPlugin replaces TWAIPlugin and becomes the
 * only path to the controller:
 *
 *   bus RX          -> ISOBUS stack + tablet
 *   stack TX        -> bus + tablet
 *   tablet TX       -> bus + ISOBUS stack (TWAI never receives its own
 *                      transmissions, so the local loop has to be explicit)
 *
 * With only the tablet connected (nothing else on the bus), bus transmits get
 * no ACK. Frames are still delivered to the tablet in that case, so the whole
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

// Starts the TCP server task and announces it over mDNS (_canlink._tcp, port
// 30000). Call after wifi_setup() and CANHardwareInterface::start().
void begin();

// True while bus/tablet frames are waiting to be handed to the stack; the
// stack takes one frame per CANHardwareInterface::update().
bool has_pending_stack_frames();

void print_status();

// Session and counter summary for the /debug page.
void debug_status(String &out);

} // namespace canlink_bridge

#endif

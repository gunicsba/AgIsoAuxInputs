/*
 * Addressable LEDs (WS2811/SK6805, FastLED-compatible) on GPIO7.
 *
 * Shows ISOBUS status first - red while no VT is connected, green for 5 s
 * after a VT connects - and afterwards the operator's light settings from
 * the VT's "Lights" page (on/off, Rainbow/Colour/White mode, R/G/B, brightness).
 * Settings are stored in NVS so they survive a power cycle.
 */
#pragma once

#include <memory>

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

// Call once in setup(), after the VT client has been created (registers the
// button / numeric-value listeners on it).
void lights_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient);

// Call from loop() as often as possible.
void lights_update();

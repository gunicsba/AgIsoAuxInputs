/*
 * Müller AUX-O output: sends the handgrip's buttons to a Müller sprayer job
 * computer the way the original Müller joystick does (see muller_sender.h),
 * next to or instead of AUX-N.
 *
 * The job computer accepts the frames by the sender's NAME, so they come from
 * a second control function with a Müller-like NAME. It only exists while the
 * mode is on (a mode change applies after a restart), so other machines never
 * see it. The VT's "Müller" page picks the mode and each button's function
 * (in multi-page mode per page; T1-T4 then only select the page). Stored in
 * NVS. Buttons only, no analog axes.
 */
#pragma once

#include <Arduino.h>
#include <memory>

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

// Loads the settings. Call before muller_create_control_function().
void muller_load();

// Creates the second control function if the mode (this boot) is on. Call
// after the CAN hardware is started.
void muller_create_control_function();

// Registers the VT page's listeners.
void muller_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient);

// True if the AUX-N button inputs must stay released ("Müller only").
bool muller_suppresses_aux_n();

// Current button bits, at the polling rate, after aux_buttons_report() (which
// switches the page).
void muller_report(uint16_t pressedBits);

// Call from loop(), after the inputs were polled, as often as possible: sends
// presses, releases and the 200 ms repeats.
void muller_update();

// Writes unsaved settings now (before a restart).
void muller_save();

// For the /debug page.
void muller_debug_status(String &out);

/*
 * Buttons -> AUX-N inputs: layout A (Handgrip keypad) in single or
 * multi-page mode, or layout B (10 round buttons, single only).
 *
 * Single: every button is one Auxiliary Input Type 2 object (layout A: one
 * per keypad PCA9555 bit; layout B: one per button).
 * Multi: T1-T4 select page 1-4 locally. Every other button has five inputs,
 * one per page plus "*" (all pages). A press goes to the current page's
 * input if a function is assigned to it, otherwise to the "*" input.
 *
 * The input unit isn't told what's assigned to it (the VT sends Auxiliary
 * Assignment Type 2 commands to the implement), so those commands are read
 * passively off the bus to know which inputs are assigned.
 */
#pragma once

#include <memory>

#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

// Registers the layout's and mode's button inputs (and model identification
// code) with the VT client and starts listening for assignment commands.
// Call before vtClient->initialize(). multiPage only applies to layout A.
// latchingButtons (layout B): bit n set = button n is a latching switch,
// reported as a latched Boolean (on while the switch is down).
void aux_buttons_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient,
                       std::shared_ptr<isobus::InternalControlFunction> ecu,
                       bool multiPage, uint8_t layout, uint16_t latchingButtons = 0);

// Reports the current button bits (layout A: bit n = keypad PCA9555 bit n
// pressed; layout B: bit n = button n of B_BUTTON_IDS). Call at the polling
// rate; forceResync re-sends every input's value.
void aux_buttons_report(uint16_t pressedBits, bool forceResync);

// Call from loop(): pushes on-screen state (designator pointers, page
// indicators, active page) after a (re)connect or a change.
void aux_buttons_update();

// Multi-page mode: current page 0..3. Always 0 in single mode.
uint8_t aux_buttons_current_page();
bool aux_buttons_multi_page();

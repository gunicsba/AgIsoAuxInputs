/*
 * Handgrip buttons -> AUX-N inputs, in single or multi-page mode.
 *
 * Single: every PCA9555 bit is one Auxiliary Input Type 2 object.
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

// Registers the mode's button inputs (and model identification code) with
// the VT client and starts listening for assignment commands. Call before
// vtClient->initialize().
void aux_buttons_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient,
                       std::shared_ptr<isobus::InternalControlFunction> ecu,
                       bool multiPage);

// Reports the current button bits (bit n = PCA9555 bit n pressed). Call at
// the polling rate; forceResync re-sends every input's value.
void aux_buttons_report(uint16_t pressedBits, bool forceResync);

// Call from loop(): pushes on-screen state (designator pointers, page
// indicators, active page) after a (re)connect or a change.
void aux_buttons_update();

// Multi-page mode: current page 0..3. Always 0 in single mode.
uint8_t aux_buttons_current_page();
bool aux_buttons_multi_page();

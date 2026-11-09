/*
 * Layout B's button teach-in: learns which input bit each round button is
 * wired to, so a grip wired by hand works without a firmware change.
 *
 * The TEACH soft key on the main mask (or the web page) starts it. The VT
 * then shows the button to press in black, with "PRESS Y1" in the title,
 * and the next one as soon as an input that no earlier button took is
 * pressed. SKIP leaves a button unwired; CANCEL keeps the previous wiring.
 * The result is saved in NVS. No AUX input status goes out while teaching,
 * so the presses can't move an implement.
 */
#pragma once

#include <memory>
#include <stdint.h>

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

static constexpr uint8_t BUTTON_TEACH_UNWIRED = 0xFF;

// Loads the saved wiring (or the default from the pool builder) and
// registers the soft key listener. Call once in setup(), layout B only.
void button_teach_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient);

// Starts (or restarts) the teach-in. Layout B only.
void button_teach_start();

bool button_teach_active();

// Input bit of each B button (0-15 keypad PCA9555, 16-31 the brain board's
// PCA9555, 32-35 ADS1115 channel pulled low), BUTTON_TEACH_UNWIRED if none
const uint8_t *button_teach_bits();

// Call at the polling rate with every input bit (see main.cpp).
void button_teach_input(uint64_t pressedBits);

// Call from loop(): keeps the VT's prompt, highlight and soft keys in step.
void button_teach_update();

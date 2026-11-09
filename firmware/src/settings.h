/*
 * Device settings (VT "Settings" page), stored in NVS:
 * - device layout (A: Handgrip keypad, B: 10 round buttons on the brain
 *   board's PCA9555, no LEDs) and page mode (single or multi-page AUX inputs,
 *   layout A only). Both pick the object pool and AUX input set at boot, so a
 *   change only applies after a restart.
 * - per-page colour shown on the LEDs in multi-page mode.
 * - press flash: blink the LEDs on every button press.
 */
#pragma once

#include <memory>

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

// Call first in setup(): the page mode decides which pool gets loaded.
void settings_load();

// Page mode and layout (DEVICE_LAYOUT_A/B) this boot runs in (the saved ones
// may differ until a restart). Layout B is always single-page.
bool settings_multi_page();
uint8_t settings_layout();

// Saves another layout for the next boot (the web page's dropdown)
void settings_set_layout(uint8_t layout);

// Layout B: bit n set = button n is a latching switch (AUX-N latched
// boolean) rather than a momentary button, as this boot runs (the INPUTS
// page's changes apply after a restart: the input type is in the pool)
uint16_t settings_b_latching();

// Colour of page 0..3 as 0xRRGGBB, at full brightness.
uint32_t settings_page_colour(uint8_t page);

bool settings_press_flash();

void settings_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient);
void settings_update();

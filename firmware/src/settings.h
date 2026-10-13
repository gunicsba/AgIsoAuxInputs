/*
 * Device settings (VT "Settings" page), stored in NVS:
 * - page mode: single or multi-page AUX inputs. Picks the object pool and
 *   AUX input set at boot, so a change only applies after a restart.
 * - per-page colour shown on the LEDs in multi-page mode.
 * - press flash: blink the LEDs on every button press.
 */
#pragma once

#include <memory>

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

// Call first in setup(): the page mode decides which pool gets loaded.
void settings_load();

// Page mode this boot runs in (the saved one may differ until a restart).
bool settings_multi_page();

// Colour of page 0..3 as 0xRRGGBB, at full brightness.
uint32_t settings_page_colour(uint8_t page);

bool settings_press_flash();

void settings_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient);
void settings_update();

/*
 * TEST (joystick_seeder env, ENABLE_TRITON_POOL): the NeptuneGPS_Triton
 * plough controller's VT object pool as a third working set on the board,
 * with its own NAME, address and VT client - the pool only, no plough
 * behind it. Its pages open from their soft keys, Wider/Narrower (soft keys
 * or their AUX-N functions, e.g. on our joystick) step the Offset value,
 * and every key and AUX event is logged to /debug.
 */
#pragma once

#include <Arduino.h>
#include <memory>

#include "isobus/isobus/can_partnered_control_function.hpp"

namespace triton_ws {

// Call once in setup(), after setupIsobus(): the VT partner is the
// joystick's (AgIsoStack binds an address claim to one partner only).
void setup(std::shared_ptr<isobus::PartneredControlFunction> vt);

// Call from loop() as often as possible.
void update();

void debug_status(String &out);

} // namespace triton_ws

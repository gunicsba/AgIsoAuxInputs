/*
 * When to send which Müller AUX-O status frame - no hardware or stack
 * dependencies, so tools/test_muller_sender.cpp can run it on a PC.
 *
 * Frame (VT to ECU PGN 0xE600, global, priority 7), as measured on the real
 * joystick: 21 nn FF FF cc 00 ss FF
 *   nn = function 0..23, cc = press counter,
 *   ss = 0 released, 1 pressed (once), 2 held (every 200 ms)
 *
 * The implement stops a held function when the 200 ms repeats stop. So the
 * repeats must never continue on stale input: update() is called from the
 * main loop only, with the buttons the same loop reads, and a hold that was
 * interrupted (bus or address lost, loop stalled) ends with "released" and
 * stays off until the operator lets the button go and presses it again.
 */
#pragma once

#include <stdint.h>

static constexpr uint8_t MULLER_NUM_FUNCTIONS = 24;
static constexpr uint8_t MULLER_NO_FUNCTION = 0xFF;
static constexpr uint8_t MULLER_NUM_BUTTONS = 16;

class MullerSender {
public:
  static constexpr uint32_t HELD_PERIOD_MS = 200;
  // A gap the implement may already have taken as a release (ISO 11783-6
  // J.7.9: 300 ms)
  static constexpr uint32_t HOLD_LOST_MS = 300;
  static constexpr uint8_t RELEASED = 0, PRESSED = 1, HELD = 2;

  // Current buttons (bit n = button n pressed). A button keeps the function
  // it was pressed with until it's released.
  void report(uint16_t pressedBits, const uint8_t *map) {
    const uint16_t newlyPressed = pressedBits & ~buttons;
    for (uint8_t b = 0; b < MULLER_NUM_BUTTONS; b++) {
      if (newlyPressed & (1u << b))
        function[b] = map[b] < MULLER_NUM_FUNCTIONS ? map[b] : MULLER_NO_FUNCTION;
    }
    buttons = pressedBits;
    inhibited &= pressedBits;
  }

  // ready: the control function may send now. send(const uint8_t frame[8])
  // returns false if the frame couldn't be queued (retried next time).
  template <typename Send> void update(uint32_t now, bool ready, Send send) {
    if (!ready) {
      inhibited |= buttons;
      return;
    }
    // Function -> buttons holding it (several may map to the same function)
    uint16_t heldBy[MULLER_NUM_FUNCTIONS] = {};
    for (uint8_t b = 0; b < MULLER_NUM_BUTTONS; b++) {
      if ((buttons & ~inhibited & (1u << b)) && function[b] != MULLER_NO_FUNCTION)
        heldBy[function[b]] |= 1u << b;
    }
    for (uint8_t nn = 0; nn < MULLER_NUM_FUNCTIONS; nn++) {
      if (active[nn] && heldBy[nn] && now - lastSent[nn] > HOLD_LOST_MS) {
        inhibited |= heldBy[nn];
        heldBy[nn] = 0;
      }
      if (heldBy[nn] && !active[nn]) {
        if (transmit(nn, PRESSED, counter[nn] + 1, send)) {
          counter[nn]++;
          active[nn] = true;
          lastSent[nn] = now;
        }
      } else if (!heldBy[nn] && active[nn]) {
        if (transmit(nn, RELEASED, counter[nn], send))
          active[nn] = false;
      } else if (active[nn] && now - lastSent[nn] >= HELD_PERIOD_MS) {
        if (transmit(nn, HELD, counter[nn], send))
          lastSent[nn] = now;
      }
    }
  }

  bool anyButtonDown() const { return buttons != 0; }

private:
  template <typename Send>
  static bool transmit(uint8_t nn, uint8_t state, uint8_t count, Send &send) {
    const uint8_t frame[8] = {0x21, nn, 0xFF, 0xFF, count, 0x00, state, 0xFF};
    return send(frame);
  }

  uint16_t buttons = 0;
  uint16_t inhibited = 0; // must be released before they count again
  uint8_t function[MULLER_NUM_BUTTONS] = {};
  bool active[MULLER_NUM_FUNCTIONS] = {}; // pressed/held was sent, released not yet
  uint8_t counter[MULLER_NUM_FUNCTIONS] = {};
  uint32_t lastSent[MULLER_NUM_FUNCTIONS] = {};
};

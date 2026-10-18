#include "aux_buttons.h"

#include <Arduino.h>

#include "debug_log.h"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "lights.h"
#include "object_pool/object_pool_ids.h"

// Main mask button rectangle colours (VT palette index).
static constexpr uint8_t COLOUR_PRESSED = 10;     // lime
static constexpr uint8_t COLOUR_RELEASED = 1;     // white, as in the pool
static constexpr uint8_t COLOUR_ACTIVE_PAGE = 14; // yellow: selected T button
static constexpr uint8_t COLOUR_INDICATOR_ON = 0; // black page indicator square

static constexpr uint8_t ALL_PAGES_LAYER = NUM_PAGES; // the "*" input
static constexpr uint8_t AUX_POINTER_ATTRIBUTE_OBJECT_ID = 2;

static std::shared_ptr<isobus::VirtualTerminalClient> vt;
static std::shared_ptr<isobus::InternalControlFunction> ecu;
static bool multi = false;

// --- AUX-N input objects -----------------------------------------------------

// ISO 11783-6 Table J.5, type 2 (non-latching Boolean): value 1 is 1 on the
// press and 2 while held; value 2 counts presses (off -> not off) only.
static constexpr uint8_t AUX_RELEASED = 0;
static constexpr uint8_t AUX_PRESSED = 1;
static constexpr uint8_t AUX_HELD = 2;
static constexpr uint32_t AUX_HELD_AFTER_MS = 200;

struct AuxInput {
  uint16_t objectId;
  uint8_t state;      // AUX_RELEASED / AUX_PRESSED / AUX_HELD
  uint16_t transitions;
  uint32_t pressedAt; // millis()
};
// Multi mode: the paged inputs, then the single-mode inputs of the
// non-paged bits (T1-T4, spares). Those exist only so a restored single-mode
// assignment always finds its input; they always report released.
static constexpr size_t MAX_INPUTS =
    NUM_PAGED_BUTTONS * PAGED_LAYERS + NUM_AUX_BUTTONS - NUM_PAGED_BUTTONS;
static AuxInput inputs[MAX_INPUTS];
static size_t inputCount = 0;

// A held input is passed on every poll: the (patched) VT client repeats it
// every 200 ms, since an implement takes a held input as released after
// 300 ms without a status message (J.7.9).
static void sendInput(AuxInput &input, bool pressed, bool forceResync) {
  const uint32_t now = millis();
  uint8_t state = AUX_RELEASED;
  if (pressed && input.state == AUX_RELEASED) {
    state = AUX_PRESSED;
    input.pressedAt = now;
    input.transitions++;
  } else if (pressed) {
    state = (now - input.pressedAt < AUX_HELD_AFTER_MS) ? AUX_PRESSED : AUX_HELD;
  }
  const bool changed = state != input.state;
  input.state = state;
  if (!changed && !forceResync && state != AUX_HELD)
    return;
  vt->update_auxiliary_input(input.objectId, state, input.transitions);
}

// --- Assignment tracking -----------------------------------------------------
// Auxiliary Assignment Type 2 command (VT -> implement, 14 bytes, so it
// travels as a TP session): [0]=0x24, [1..8]=input unit NAME,
// [9]=preferred flag (bit 7) + function type, [10..11]=input object ID
// (0xFFFF = unassign), [12..13]=function object ID.

struct Assignment {
  uint8_t ownerAddress; // implement that owns the function
  uint16_t functionId;
  uint16_t inputId;
};
static constexpr size_t MAX_ASSIGNMENTS = 64;
static Assignment assignments[MAX_ASSIGNMENTS];
static size_t assignmentCount = 0;
static bool assignmentsChanged = false;

static bool isAssigned(uint16_t inputId) {
  for (size_t i = 0; i < assignmentCount; i++) {
    if (assignments[i].inputId == inputId)
      return true;
  }
  return false;
}

static void removeAssignments(uint8_t owner, bool allFunctions,
                              uint16_t functionId) {
  size_t kept = 0;
  for (size_t i = 0; i < assignmentCount; i++) {
    const Assignment &a = assignments[i];
    if (a.ownerAddress == owner && (allFunctions || a.functionId == functionId))
      continue;
    assignments[kept++] = a;
  }
  if (kept != assignmentCount)
    assignmentsChanged = true;
  assignmentCount = kept;
}

static void handleAssignmentCommand(uint8_t owner, const uint8_t *msg) {
  uint64_t name = 0;
  for (int i = 8; i >= 1; i--)
    name = (name << 8) | msg[i];
  uint8_t functionType = msg[9] & 0x1F;
  uint16_t inputId = msg[10] | (msg[11] << 8);
  uint16_t functionId = msg[12] | (msg[13] << 8);

  if (name == UINT64_MAX && functionType == 0x1F &&
      functionId == isobus::NULL_OBJECT_ID) {
    removeAssignments(owner, true, 0);
    debug_log("[AUX-N] VT -> addr %u: all functions unassigned", owner);
    return;
  }
  // A function has at most one input: drop its old assignment either way.
  removeAssignments(owner, false, functionId);
  if (inputId == isobus::NULL_OBJECT_ID || name != ecu->get_NAME().get_full_name())
    return;
  if (assignmentCount < MAX_ASSIGNMENTS) {
    assignments[assignmentCount++] = {owner, functionId, inputId};
    assignmentsChanged = true;
  }
  debug_log("[AUX-N] VT -> addr %u: function %u assigned to our input %u", owner, functionId,
            inputId);
}

// Passive TP reassembly of VT -> implement sessions carrying the 14-byte
// command. Our stack only reassembles sessions addressed to us.
struct TpSniff {
  bool active;
  uint8_t source, destination;
  uint8_t packets;
  uint8_t data[14];
};
static TpSniff sniff[4];

static constexpr uint8_t PF_TP_CM = 0xEC;
static constexpr uint8_t PF_TP_DT = 0xEB;
static constexpr uint8_t TP_CM_RTS = 16;
static constexpr uint32_t PGN_VT_TO_ECU = 0xE600;
static constexpr uint32_t PGN_ECU_TO_VT = 0xE700;
static constexpr uint8_t PF_VT_TO_ECU = 0xE6;
static constexpr uint8_t PF_ECU_TO_VT = 0xE7;
static constexpr uint8_t VT_AUX_ASSIGNMENT_TYPE_2 = 0x24;
static constexpr uint8_t VT_PREFERRED_ASSIGNMENT = 0x22;
static constexpr uint8_t VT_AUX_INPUT_MAINTENANCE = 0x23;
static constexpr uint8_t VT_AUX_INPUT_STATUS_ENABLE = 0x25;

// The AUX-N handshake as seen on the bus, logged to /debug: an implement's
// Preferred Assignment (J.7.7) and the VT's response (J.7.8) - the VT allows
// no manual assignment to an implement's functions before a valid one -,
// the VT enabling our inputs (J.7.11), and the other auxiliary input units
// (J.7.10).
struct PreferredSniff {
  bool active;
  uint8_t source;
  uint16_t size;
};
static PreferredSniff preferredSniff[4];
static uint8_t inputUnitStatus[256]; // last maintenance status per source, 0xFF = unseen

static void logAuxHandshake(uint8_t pf, uint8_t da, uint8_t sa, const uint8_t *d) {
  if (pf == PF_VT_TO_ECU && d[0] == VT_PREFERRED_ASSIGNMENT) {
    debug_log("[AUX-N] VT -> addr %u: Preferred Assignment response, error 0x%02X "
              "(1 input unit, 2 function ID, 4 input ID, 8 duplicate, 16 other), function %u",
              da, d[1], (unsigned)(d[2] | (d[3] << 8)));
  } else if (pf == PF_ECU_TO_VT && d[0] == VT_PREFERRED_ASSIGNMENT) {
    debug_log("[AUX-N] addr %u -> VT: Preferred Assignment, %u input units", sa, d[1]);
  } else if (pf == PF_VT_TO_ECU && d[0] == VT_AUX_INPUT_STATUS_ENABLE && ecu->get_address_valid() &&
             da == ecu->get_address()) {
    debug_log("[AUX-N] VT %s input %u", (d[3] & 1) ? "enables" : "disables",
              (unsigned)(d[1] | (d[2] << 8)));
  } else if (pf == PF_ECU_TO_VT && da == 0xFF && d[0] == VT_AUX_INPUT_MAINTENANCE &&
             inputUnitStatus[sa] != d[3]) {
    inputUnitStatus[sa] = d[3];
    debug_log("[AUX-N] addr %u: auxiliary input unit, model %u, %s", sa,
              (unsigned)(d[1] | (d[2] << 8)), d[3] == 1 ? "ready" : "initializing");
  }
}

static void onFrame(const isobus::CANMessageFrame &frame) {
  if (!frame.isExtendedFrame || frame.dataLength != 8)
    return;
  uint8_t pf = (frame.identifier >> 16) & 0xFF;
  uint8_t da = (frame.identifier >> 8) & 0xFF;
  uint8_t sa = frame.identifier & 0xFF;
  const uint8_t *d = frame.data;
  logAuxHandshake(pf, da, sa, d);

  if (pf == PF_TP_CM && d[0] == TP_CM_RTS) {
    uint16_t size = d[1] | (d[2] << 8);
    uint32_t pgn = d[5] | (d[6] << 8) | ((uint32_t)d[7] << 16);
    // A Preferred Assignment naming input units takes a TP session; only its
    // first packet is needed (function code, number of input units).
    if (pgn == PGN_ECU_TO_VT && size <= 255) {
      for (auto &p : preferredSniff) {
        if (!p.active || p.source == sa) {
          p = {true, sa, size};
          break;
        }
      }
    }
    if (pgn != PGN_VT_TO_ECU || size != sizeof(TpSniff::data))
      return;
    TpSniff *slot = nullptr;
    for (auto &s : sniff) {
      if (s.active && s.source == sa && s.destination == da) {
        slot = &s; // restarted session
        break;
      }
      if (!s.active && !slot)
        slot = &s;
    }
    if (slot)
      *slot = {true, sa, da, d[3], {}};
  } else if (pf == PF_TP_DT) {
    for (auto &p : preferredSniff) {
      if (p.active && p.source == sa && d[0] == 1) {
        p.active = false;
        if (d[1] == VT_PREFERRED_ASSIGNMENT)
          debug_log("[AUX-N] addr %u -> VT: Preferred Assignment, %u input units (%u bytes)",
                    sa, d[2], p.size);
      }
    }
    for (auto &s : sniff) {
      if (!s.active || s.source != sa || s.destination != da)
        continue;
      uint8_t seq = d[0];
      if (seq < 1 || seq > s.packets) {
        s.active = false;
        return;
      }
      for (int i = 0; i < 7; i++) {
        size_t offset = (seq - 1) * 7 + i;
        if (offset < sizeof(s.data))
          s.data[offset] = d[1 + i];
      }
      if (seq == s.packets) {
        s.active = false;
        if (s.data[0] == VT_AUX_ASSIGNMENT_TYPE_2)
          handleAssignmentCommand(s.destination, s.data);
      }
      return;
    }
  }
}

// --- Pages and on-screen state ------------------------------------------------

static uint8_t currentPage = 0;
static uint16_t lastPressedBits = 0;
static uint8_t effectiveLayer[16];     // per paged bit
static uint8_t shownLayer[16];         // what the VT pointer/indicators show
static uint8_t shownIndicators[16];    // bitmask of filled page squares
static bool screenDirty = true;
static bool wasConnected = false;

// b = index into PAGED_BUTTON_BITS / PAGED_AUX_IDS
static uint8_t layerFor(uint8_t b) {
  if (isAssigned(PAGED_AUX_IDS[b][currentPage]))
    return currentPage;
  return ALL_PAGES_LAYER;
}

static uint8_t indicatorsFor(uint8_t b) {
  if (isAssigned(PAGED_AUX_IDS[b][currentPage]))
    return 1u << currentPage;             // -X--
  if (isAssigned(PAGED_AUX_IDS[b][ALL_PAGES_LAYER]))
    return (1u << NUM_PAGES) - 1;         // XXXX
  return 0;                               // ---- nothing assigned
}

static bool isPageSelectBit(uint8_t bit, uint8_t *page) {
  for (uint8_t p = 0; p < NUM_PAGES; p++) {
    if (PAGE_SELECT_BITS[p] == bit) {
      if (page)
        *page = p;
      return true;
    }
  }
  return false;
}

static uint8_t buttonBaseColour(uint8_t bit) {
  uint8_t page;
  if (multi && isPageSelectBit(bit, &page) && page == currentPage)
    return COLOUR_ACTIVE_PAGE;
  return COLOUR_RELEASED;
}

static void setButtonFill(uint8_t bit, uint8_t colour) {
  if (!(JOY_BUTTONS_ON_SCREEN & (1u << bit)) || !vt->get_is_connected())
    return;
  vt->send_change_fill_attributes(
      JOY_BUTTON_FILL_BASE + bit,
      isobus::VirtualTerminalClient::FillType::
          FillWithSpecifiedColourInFillColourAttribute,
      colour, isobus::NULL_OBJECT_ID);
}

// --- Public ------------------------------------------------------------------

void aux_buttons_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient,
                       std::shared_ptr<isobus::InternalControlFunction> internalEcu,
                       bool multiPage) {
  vt = vtClient;
  ecu = internalEcu;
  multi = multiPage;

  if (multi) {
    uint16_t pagedBits = 0;
    for (uint8_t b = 0; b < NUM_PAGED_BUTTONS; b++) {
      for (uint8_t layer = 0; layer < PAGED_LAYERS; layer++)
        inputs[inputCount++] = {PAGED_AUX_IDS[b][layer], AUX_RELEASED, 0, 0};
      effectiveLayer[PAGED_BUTTON_BITS[b]] = ALL_PAGES_LAYER;
      pagedBits |= 1u << PAGED_BUTTON_BITS[b];
    }
    for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++) {
      if (!(pagedBits & (1u << i)))
        inputs[inputCount++] = {AUX_BUTTON_IDS[i], AUX_RELEASED, 0, 0};
    }
  } else {
    for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++)
      inputs[inputCount++] = {AUX_BUTTON_IDS[i], AUX_RELEASED, 0, 0};
  }
  for (size_t i = 0; i < inputCount; i++)
    vt->add_auxiliary_input_object_id(inputs[i].objectId);
  vt->set_auxiliary_input_model_identification_code(multi ? MODEL_ID_MULTI
                                                          : MODEL_ID_SINGLE);

  memset(inputUnitStatus, 0xFF, sizeof(inputUnitStatus));
  isobus::CANHardwareInterface::get_can_frame_received_event_dispatcher()
      .add_listener(onFrame);
}

void aux_buttons_report(uint16_t pressedBits, bool forceResync) {
  uint16_t newlyPressed = pressedBits & ~lastPressedBits;
  uint16_t changed = pressedBits ^ lastPressedBits;
  lastPressedBits = pressedBits;

  if (newlyPressed)
    lights_button_pressed();

  if (multi) {
    for (uint8_t p = 0; p < NUM_PAGES; p++) {
      if ((newlyPressed & (1u << PAGE_SELECT_BITS[p])) && p != currentPage) {
        currentPage = p;
        screenDirty = true;
        Serial.printf("[AUX-N] page %u\n", p + 1);
      }
    }
    size_t n = 0;
    for (uint8_t b = 0; b < NUM_PAGED_BUTTONS; b++) {
      uint8_t bit = PAGED_BUTTON_BITS[b];
      bool pressed = pressedBits & (1u << bit);
      // Keep a held button on the input it started on, so a page switch or
      // an assignment change mid-press can't leave a function stuck on.
      if (!pressed)
        effectiveLayer[bit] = layerFor(b);
      for (uint8_t layer = 0; layer < PAGED_LAYERS; layer++)
        sendInput(inputs[n++], pressed && layer == effectiveLayer[bit],
                  forceResync);
    }
    while (n < inputCount)
      sendInput(inputs[n++], false, forceResync);
  } else {
    for (uint8_t i = 0; i < NUM_AUX_BUTTONS; i++)
      sendInput(inputs[i], pressedBits & (1u << i), forceResync);
  }

  for (uint8_t bit = 0; bit < 16; bit++) {
    if (changed & (1u << bit))
      setButtonFill(bit, (pressedBits & (1u << bit)) ? COLOUR_PRESSED
                                                     : buttonBaseColour(bit));
  }
}

void aux_buttons_update() {
  bool connected = vt->get_is_connected();
  if (connected && !wasConnected) {
    // A (re)connected VT shows the pool's initial state.
    for (uint8_t bit = 0; bit < 16; bit++)
      shownLayer[bit] = ALL_PAGES_LAYER, shownIndicators[bit] = 0;
    screenDirty = true;
    for (uint8_t bit = 0; bit < 16; bit++) {
      bool pressed = lastPressedBits & (1u << bit);
      if (pressed || buttonBaseColour(bit) != COLOUR_RELEASED)
        setButtonFill(bit, pressed ? COLOUR_PRESSED : buttonBaseColour(bit));
    }
  }
  wasConnected = connected;
  if (!multi || !connected || !(screenDirty || assignmentsChanged))
    return;

  if (screenDirty) {
    // Page change: move the yellow highlight to the new T button.
    for (uint8_t p = 0; p < NUM_PAGES; p++) {
      uint8_t bit = PAGE_SELECT_BITS[p];
      if (!(lastPressedBits & (1u << bit)))
        setButtonFill(bit, buttonBaseColour(bit));
    }
  }
  screenDirty = false;
  assignmentsChanged = false;

  for (uint8_t b = 0; b < NUM_PAGED_BUTTONS; b++) {
    uint8_t bit = PAGED_BUTTON_BITS[b];
    uint8_t layer = layerFor(b);
    if (layer != shownLayer[bit]) {
      vt->send_change_attribute(AUX_POINTER_BASE + bit,
                                AUX_POINTER_ATTRIBUTE_OBJECT_ID,
                                (uint32_t)PAGED_AUX_IDS[b][layer]);
      shownLayer[bit] = layer;
    }
    uint8_t indicators = indicatorsFor(b);
    uint8_t changedSquares = indicators ^ shownIndicators[bit];
    for (uint8_t p = 0; p < NUM_PAGES; p++) {
      if (!(changedSquares & (1u << p)))
        continue;
      vt->send_change_fill_attributes(
          PAGE_IND_FILL_BASE + bit * NUM_PAGES + p,
          isobus::VirtualTerminalClient::FillType::
              FillWithSpecifiedColourInFillColourAttribute,
          (indicators & (1u << p)) ? COLOUR_INDICATOR_ON : COLOUR_RELEASED,
          isobus::NULL_OBJECT_ID);
    }
    shownIndicators[bit] = indicators;
  }
}

uint8_t aux_buttons_current_page() { return currentPage; }

bool aux_buttons_multi_page() { return multi; }

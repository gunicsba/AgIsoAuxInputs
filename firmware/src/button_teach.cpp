#include "button_teach.h"

#include <Arduino.h>
#include <Preferences.h>

#include "debug_log.h"
#include "object_pool/object_pool_ids.h"

static constexpr uint8_t COLOUR_HIGHLIGHT = 0; // black: the button to press
static constexpr uint8_t NUM_INPUT_BITS = 36; // see main.cpp: PCA9555s, then ADS1115 channels

static std::shared_ptr<isobus::VirtualTerminalClient> vt;
static Preferences prefs;

static uint8_t bits[NUM_B_BUTTONS];
static uint8_t previousBits[NUM_B_BUTTONS]; // restored by CANCEL
static bool active = false;
static uint8_t teachIndex = 0;            // button being taught
static uint64_t lastPressed = 0;
static bool waitForRelease = false;  // after the last button: until it's let go

// What the VT shows, so a full TX queue just retries on the next loop
static bool wasConnected = false;
static int8_t shownHighlight = -1;   // button shown black, -1 none
static bool shownTeachKeys = false;
static int8_t shownPrompt = -1;      // button named in the title, -1 the normal title

static void save() {
  prefs.putBytes("bBits", bits, sizeof(bits));
  String wiring;
  for (uint8_t i = 0; i < NUM_B_BUTTONS; i++) {
    wiring += String(i ? ", " : "") + B_BUTTON_LABELS[i] + "=";
    wiring += bits[i] == BUTTON_TEACH_UNWIRED ? String("-") : String(bits[i]);
  }
  debug_log("[TEACH] Saved: %s", wiring.c_str());
}

static void finish() {
  active = false;
  waitForRelease = true;
  save();
}

static void advance() {
  if (++teachIndex >= NUM_B_BUTTONS)
    finish();
}

static void onSoftKey(const isobus::VirtualTerminalClient::VTKeyEvent &event) {
  if (event.keyEvent !=
      isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
    return;
  if (event.objectID == KEY_TEACH) {
    button_teach_start();
  } else if (active && event.objectID == KEY_TEACH_SKIP) {
    debug_log("[TEACH] %s skipped (not wired)", B_BUTTON_LABELS[teachIndex]);
    bits[teachIndex] = BUTTON_TEACH_UNWIRED;
    advance();
  } else if (active && event.objectID == KEY_TEACH_CANCEL) {
    memcpy(bits, previousBits, sizeof(bits));
    active = false;
    debug_log("[TEACH] Cancelled, wiring unchanged");
  }
}

void button_teach_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient) {
  vt = vtClient;
  memcpy(bits, B_BUTTON_INPUT_BITS, sizeof(bits));
  prefs.begin("teach", false);
  uint8_t stored[NUM_B_BUTTONS];
  if (prefs.getBytesLength("bBits") == sizeof(stored) &&
      prefs.getBytes("bBits", stored, sizeof(stored)) == sizeof(stored)) {
    bool valid = true;
    for (uint8_t bit : stored)
      valid &= bit < NUM_INPUT_BITS || bit == BUTTON_TEACH_UNWIRED;
    if (valid)
      memcpy(bits, stored, sizeof(bits));
  }
  vt->get_vt_soft_key_event_dispatcher().add_listener(onSoftKey);
}

void button_teach_start() {
  if (!vt)
    return; // not layout B
  if (!active)
    memcpy(previousBits, bits, sizeof(bits));
  active = true;
  teachIndex = 0;
  debug_log("[TEACH] Started: press each highlighted button in turn");
  if (vt->get_is_connected())
    vt->send_change_active_mask(WORKING_SET, MAIN_MASK);
}

bool button_teach_active() { return active || waitForRelease; }

const uint8_t *button_teach_bits() { return bits; }

void button_teach_input(uint64_t pressed) {
  const uint64_t newlyPressed = pressed & ~lastPressed;
  lastPressed = pressed;
  if (waitForRelease) {
    uint64_t wired = 0;
    for (uint8_t bit : bits)
      if (bit < NUM_INPUT_BITS)
        wired |= 1ull << bit;
    waitForRelease = (pressed & wired) != 0;
  }
  if (!active || !newlyPressed)
    return;
  // An input an earlier button of this run took is a bounce or a mistake
  uint64_t taken = 0;
  for (uint8_t i = 0; i < teachIndex; i++)
    if (bits[i] < NUM_INPUT_BITS)
      taken |= 1ull << bits[i];
  const uint64_t candidates = newlyPressed & ~taken;
  if (!candidates)
    return;
  bits[teachIndex] = __builtin_ctzll(candidates);
  debug_log("[TEACH] %s = input bit %u", B_BUTTON_LABELS[teachIndex], bits[teachIndex]);
  advance();
}

static bool sendFill(int8_t button, uint8_t colour) {
  return vt->send_change_fill_attributes(
      JOY_BUTTON_FILL_BASE + button,
      isobus::VirtualTerminalClient::FillType::FillWithSpecifiedColourInFillColourAttribute,
      colour, isobus::NULL_OBJECT_ID);
}

static bool sendTitle(int8_t button) {
  char text[TITLE_TEXT_LENGTH + 1];
  if (button >= 0)
    snprintf(text, sizeof(text), "PRESS %s", B_BUTTON_LABELS[button]);
  else
    snprintf(text, sizeof(text), "%s", MAIN_TITLE_TEXT);
  // Centred in the pool string's length, like the builder pads the title
  const size_t length = strlen(text);
  const size_t left = (TITLE_TEXT_LENGTH - length) / 2;
  char padded[TITLE_TEXT_LENGTH + 1];
  memset(padded, ' ', TITLE_TEXT_LENGTH);
  padded[TITLE_TEXT_LENGTH] = '\0';
  memcpy(padded + left, text, length);
  return vt->send_change_string_value(STR_MAIN_TITLE, padded);
}

void button_teach_update() {
  if (!vt)
    return;
  const bool connected = vt->get_is_connected();
  if (connected && !wasConnected) {
    // A (re)connected VT shows the pool's initial state
    shownHighlight = shownPrompt = -1;
    shownTeachKeys = false;
    if (active)
      vt->send_change_active_mask(WORKING_SET, MAIN_MASK);
  }
  wasConnected = connected;
  if (!connected)
    return;

  const int8_t want = active ? teachIndex : -1;
  if (shownHighlight != want) {
    if (shownHighlight >= 0) {
      if (!sendFill(shownHighlight, B_BUTTON_COLOURS[shownHighlight]))
        return;
      shownHighlight = -1;
    }
    if (want >= 0) {
      if (!sendFill(want, COLOUR_HIGHLIGHT))
        return;
      shownHighlight = want;
    }
  }
  if (shownTeachKeys != active) {
    if (!vt->send_change_softkey_mask(isobus::VirtualTerminalClient::MaskType::DataMask, MAIN_MASK,
                                      active ? TEACH_SOFT_KEY_MASK : MAIN_SOFT_KEY_MASK))
      return;
    shownTeachKeys = active;
  }
  if (shownPrompt != want && sendTitle(want))
    shownPrompt = want;
}

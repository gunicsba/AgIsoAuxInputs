#include "muller.h"

#include <Preferences.h>

#include "aux_buttons.h"
#include "debug_log.h"
#include "driver/twai.h"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_parameter_group_number_request_protocol.hpp"
#include "muller_sender.h"
#include "object_pool/object_pool_ids.h"
#include "settings.h"

static_assert(NUM_AUX_BUTTONS == MULLER_NUM_BUTTONS, "one map entry per button");

static constexpr uint32_t SAVE_DELAY_MS = 2000; // batch NVS writes while editing
static constexpr uint32_t MAINTENANCE_PERIOD_MS = 1000;
static constexpr uint32_t TWAI_ERROR_PASSIVE_LIMIT = 128;
static constexpr uint32_t DEFAULT_IDENTITY = 0x1ABCDE; // the real one is 0x10C71D
static constexpr uint32_t MAX_IDENTITY = 0x1FFFFF;
static constexpr uint8_t DEFAULT_ADDRESS = 0x90;
static constexpr uint8_t MIN_ADDRESS = 0x80, MAX_ADDRESS = 0xEB; // self-configurable range

enum MullerMode : uint8_t { MODE_OFF = 0, MODE_MULLER_ONLY = 1, MODE_AUX_N_AND_MULLER = 2 };

struct MullerSettings {
  uint8_t mode;
  uint8_t map[MULLER_NUM_BUTTONS]; // function 0..23 or MULLER_NO_FUNCTION
};
using PageMaps = uint8_t[NUM_PAGES][MULLER_NUM_BUTTONS];

static Preferences prefs;
static MullerSettings settings; // its map is the single-page mode's
static PageMaps pageMaps;       // multi-page mode
static bool multi = false;
static uint8_t shownPage = 0;   // multi-page mode: page on the VT's Müller page
static uint8_t lastPage = 0;
static uint8_t activeMode = MODE_OFF; // what this boot runs
static uint32_t identity = DEFAULT_IDENTITY;
static uint8_t preferredAddress = DEFAULT_ADDRESS;

static std::shared_ptr<isobus::InternalControlFunction> cf;
static std::shared_ptr<isobus::VirtualTerminalClient> vt;
static MullerSender sender;
static uint32_t lastMaintenance = 0;

static bool dirty = false;
static uint32_t changedAt = 0;
static bool vtWasConnected = false;
static bool vtSynced = false;

static bool isValidMap(const uint8_t *map) {
  for (uint8_t b = 0; b < MULLER_NUM_BUTTONS; b++) {
    if (map[b] >= MULLER_NUM_FUNCTIONS && map[b] != MULLER_NO_FUNCTION)
      return false;
  }
  return true;
}

static bool isValid(const MullerSettings &s) {
  return s.mode <= MODE_AUX_N_AND_MULLER && isValidMap(s.map);
}

static bool isValid(const PageMaps &maps) {
  for (const auto &map : maps) {
    if (!isValidMap(map))
      return false;
  }
  return true;
}

// In multi-page mode T1-T4 only select the page.
static bool isPageSelect(uint8_t bit) {
  if (!multi)
    return false;
  for (uint8_t select : PAGE_SELECT_BITS) {
    if (select == bit)
      return true;
  }
  return false;
}

static uint8_t *mapOf(uint8_t page) { return multi ? pageMaps[page] : settings.map; }

void muller_load() {
  settings.mode = MODE_OFF;
  memset(settings.map, MULLER_NO_FUNCTION, sizeof(settings.map));
  prefs.begin("muller", false);
  MullerSettings stored;
  if (prefs.getBytes("settings", &stored, sizeof(stored)) == sizeof(stored) && isValid(stored))
    settings = stored;
  // Until pages were assigned, every page starts with the single-page map.
  PageMaps storedPages;
  if (prefs.getBytes("pages", storedPages, sizeof(storedPages)) == sizeof(storedPages) &&
      isValid(storedPages)) {
    memcpy(pageMaps, storedPages, sizeof(pageMaps));
  } else {
    for (auto &map : pageMaps)
      memcpy(map, settings.map, sizeof(map));
  }
  multi = settings_multi_page();
  identity = prefs.getUInt("identity", DEFAULT_IDENTITY);
  if (identity > MAX_IDENTITY)
    identity = DEFAULT_IDENTITY;
  preferredAddress = prefs.getUChar("sa", DEFAULT_ADDRESS);
  if (preferredAddress < MIN_ADDRESS || preferredAddress > MAX_ADDRESS)
    preferredAddress = DEFAULT_ADDRESS;
  activeMode = settings.mode;
}

// As for the AUX-N control function (main.cpp): the claim itself is the
// answer, AgIsoStack would also send a NACK.
static bool onAddressClaimRequest(std::uint32_t, std::shared_ptr<isobus::ControlFunction>,
                                  bool &acknowledge, isobus::AcknowledgementType &, void *) {
  acknowledge = false;
  return true;
}

void muller_create_control_function() {
  if (activeMode == MODE_OFF)
    return;
  isobus::NAME name(0);
  name.set_arbitrary_address_capable(true);
  name.set_industry_group(2);
  name.set_device_class(13);
  name.set_device_class_instance(0);
  name.set_function_code(128);
  name.set_function_instance(0);
  name.set_ecu_instance(0);
  name.set_manufacturer_code(98);
  name.set_identity_number(identity);
  cf = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(name, 0,
                                                                            preferredAddress);
  if (auto requests = cf->get_pgn_request_protocol().lock()) {
    requests->register_pgn_request_callback(
        static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::AddressClaim),
        onAddressClaimRequest, nullptr);
  }
  debug_log("[MULLER] mode %u, identity 0x%06lX, preferred address 0x%02X", activeMode,
            (unsigned long)identity, preferredAddress);
}

bool muller_suppresses_aux_n() { return activeMode == MODE_MULLER_ONLY; }

void muller_report(uint16_t pressedBits) {
  for (uint8_t bit = 0; bit < MULLER_NUM_BUTTONS; bit++) {
    if (isPageSelect(bit))
      pressedBits &= ~(1u << bit);
  }
  // A held button keeps its function when the page changes (see MullerSender).
  sender.report(pressedBits, mapOf(aux_buttons_current_page()));
}

// With nobody acknowledging (cable unplugged) frames would pile up and go out
// late, as stale presses, when the bus returns. The controller is then error
// passive, with the frame it keeps retrying still queued.
static bool busUp() {
  twai_status_info_t status;
  return twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING &&
         !(status.tx_error_counter >= TWAI_ERROR_PASSIVE_LIMIT && status.msgs_to_tx > 0);
}

static bool sendGlobal(isobus::CANLibParameterGroupNumber pgn, const uint8_t *data,
                       isobus::CANIdentifier::CANPriority priority) {
  return isobus::CANNetworkManager::CANNetwork.send_can_message(
      static_cast<std::uint32_t>(pgn), data, 8, cf, nullptr, priority);
}

static bool sendStatus(const uint8_t *frame) {
  return sendGlobal(isobus::CANLibParameterGroupNumber::VirtualTerminalToECU, frame,
                    isobus::CANIdentifier::CANPriority::PriorityLowest7);
}

static void updateOutput(uint32_t now) {
  const bool ready = cf->get_address_valid() && busUp();
  sender.update(now, ready, sendStatus);

  // The real joystick sends this every second. Whether the job computer needs
  // it isn't known; tests so far were done with it.
  if (ready && now - lastMaintenance >= MAINTENANCE_PERIOD_MS) {
    static const uint8_t MAINTENANCE[8] = {0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (sendGlobal(isobus::CANLibParameterGroupNumber::ECUtoVirtualTerminal, MAINTENANCE,
                   isobus::CANIdentifier::CANPriority::PriorityDefault6))
      lastMaintenance = now;
  }
}

// --- VT page -----------------------------------------------------------------
// A button's input list has its index as value: 0 = no function, n = function
// n - 1. The mode list's index is the mode.

static uint8_t listIndex(uint8_t function) {
  return function == MULLER_NO_FUNCTION ? 0 : function + 1;
}

static bool isListedButton(uint16_t bit) {
  return bit < MULLER_NUM_BUTTONS && (JOY_BUTTONS_ON_SCREEN & (1u << bit)) && !isPageSelect(bit);
}

static bool sendMode() { return vt->send_change_numeric_value(LIST_MULLER_MODE, settings.mode); }

static bool sendButton(uint8_t bit) {
  return vt->send_change_numeric_value(LIST_MULLER_BUTTON_BASE + bit,
                                       listIndex(mapOf(shownPage)[bit]));
}

static bool syncVt() {
  bool ok = sendMode();
  if (ok && multi) // the list only exists in the multi-page pool
    ok = vt->send_change_numeric_value(LIST_MULLER_PAGE, shownPage);
  for (uint8_t bit = 0; ok && bit < MULLER_NUM_BUTTONS; bit++) {
    if (isListedButton(bit))
      ok = sendButton(bit);
  }
  return ok;
}

static void markChanged() {
  dirty = true;
  changedAt = millis();
}

static void onKey(const isobus::VirtualTerminalClient::VTKeyEvent &event) {
  if (event.objectID == KEY_MULLER &&
      event.keyEvent ==
          isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
    vt->send_change_active_mask(WORKING_SET, MULLER_MASK);
}

static void onNumericValue(
    const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event) {
  if (event.objectID == LIST_MULLER_MODE) {
    if (event.value > MODE_AUX_N_AND_MULLER) {
      sendMode();
    } else if (settings.mode != event.value) {
      settings.mode = event.value;
      debug_log("[MULLER] mode %u after the next restart", settings.mode);
      markChanged();
    }
  } else if (multi && event.objectID == LIST_MULLER_PAGE) {
    if (event.value < NUM_PAGES)
      shownPage = event.value;
    vtSynced = false; // show that page's functions
  } else if (event.objectID >= LIST_MULLER_BUTTON_BASE &&
             isListedButton(event.objectID - LIST_MULLER_BUTTON_BASE)) {
    const uint8_t bit = event.objectID - LIST_MULLER_BUTTON_BASE;
    if (event.value > MULLER_NUM_FUNCTIONS) {
      sendButton(bit); // e.g. 255, "nothing chosen"
      return;
    }
    const uint8_t function = event.value == 0 ? MULLER_NO_FUNCTION : event.value - 1;
    uint8_t &mapped = mapOf(shownPage)[bit];
    if (mapped != function) {
      mapped = function;
      debug_log("[MULLER] page %u button %u -> function %u", shownPage + 1, bit, function);
      markChanged();
    }
  }
}

void muller_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient) {
  vt = vtClient;
  vt->get_vt_soft_key_event_dispatcher().add_listener(onKey);
  vt->get_vt_change_numeric_value_event_dispatcher().add_listener(onNumericValue);
}

void muller_save() {
  if (!dirty)
    return;
  prefs.putBytes("settings", &settings, sizeof(settings));
  if (multi)
    prefs.putBytes("pages", pageMaps, sizeof(pageMaps));
  dirty = false;
}

void muller_update() {
  const uint32_t now = millis();
  if (cf)
    updateOutput(now);

  // T1-T4 also pick the page shown on the VT's Müller page.
  const uint8_t page = aux_buttons_current_page();
  if (page != lastPage) {
    lastPage = shownPage = page;
    vtSynced = false;
  }

  const bool connected = vt->get_is_connected();
  if (connected && !vtWasConnected)
    vtSynced = false;
  vtWasConnected = connected;
  if (connected && !vtSynced)
    vtSynced = syncVt(); // retried next loop if the TX queue was full

  // A flash write blocks the loop, so not while a function may be held.
  if (dirty && now - changedAt >= SAVE_DELAY_MS && !sender.anyButtonDown())
    muller_save();
}

void muller_debug_status(String &out) {
  static const char *MODES[] = {"off", "Müller only", "AUX-N + Müller"};
  out += String("MÜLLER\n  ") + MODES[activeMode];
  if (settings.mode != activeMode)
    out += String(" (after a restart: ") + MODES[settings.mode] + ")";
  if (cf) {
    out += ", address ";
    out += cf->get_address_valid() ? "0x" + String(cf->get_address(), HEX) : String("not claimed");
    out += ", bus ";
    out += busUp() ? "up" : "down";
  }
  out += "\n";
  for (uint8_t page = 0; page < (multi ? NUM_PAGES : 1); page++) {
    out += multi ? "  page " + String(page + 1) + " functions by button bit:"
                 : String("  functions by button bit:");
    for (uint8_t bit = 0; bit < MULLER_NUM_BUTTONS; bit++) {
      const uint8_t function = mapOf(page)[bit];
      out += function == MULLER_NO_FUNCTION || isPageSelect(bit) ? String(" -")
                                                                 : " " + String(function);
    }
    out += "\n";
  }
}

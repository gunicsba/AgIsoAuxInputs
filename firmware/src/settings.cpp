#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>

#include "object_pool/object_pool_ids.h"

static constexpr uint32_t SAVE_DELAY_MS = 2000; // batch NVS writes while editing
static constexpr uint32_t RESTART_DELAY_MS = 300; // let the save/log finish

struct PresetColour {
  const char *name;
  uint32_t rgb;
};
static const PresetColour PRESETS[] = {
    {"GREEN", 0x00FF00}, {"BLUE", 0x0000FF},  {"YELLOW", 0xFFC000},
    {"MAGENTA", 0xFF00FF}, {"CYAN", 0x00FFFF}, {"ORANGE", 0xFF4000},
    {"WHITE", 0xFFFFFF}, {"RED", 0xFF0000},
};
static constexpr uint8_t NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

struct DeviceSettings {
  uint8_t multiPage;
  uint8_t pageColour[NUM_PAGES]; // index into PRESETS
  uint8_t pressFlash;
};

static Preferences prefs;
static std::shared_ptr<isobus::VirtualTerminalClient> vt;
static DeviceSettings settings = {0, {0, 1, 2, 3}, 1};
static bool activeMultiPage = false; // what this boot runs
static bool dirty = false;
static uint32_t changedAt = 0;
static uint32_t restartAt = 0;
static bool vtWasConnected = false;
static bool vtSynced = false;

// Pads to the pool string's length so the VT has room for every variant.
static bool sendPadded(uint16_t objectId, int length, const char *text) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%-*s", length, text);
  return vt->send_change_string_value(objectId, buffer);
}

static bool sendPageMode() {
  bool pending = (bool)settings.multiPage != activeMultiPage;
  return sendPadded(STR_PAGE_MODE, PAGE_MODE_TEXT_LENGTH,
                    settings.multiPage ? "PAGES: MULTI" : "PAGES: SINGLE") &&
         sendPadded(STR_RESTART_HINT, RESTART_HINT_TEXT_LENGTH,
                    pending ? "RESTART TO APPLY" : "");
}

static bool sendPageColour(uint8_t page) {
  char text[24];
  snprintf(text, sizeof(text), "PAGE %u: %s", page + 1,
           PRESETS[settings.pageColour[page]].name);
  return sendPadded(STR_PAGE_COLOUR_BASE + page, PAGE_COLOUR_TEXT_LENGTH, text);
}

static bool sendPressFlash() {
  return sendPadded(STR_PRESS_FLASH, PRESS_FLASH_TEXT_LENGTH,
                    settings.pressFlash ? "PRESS FLASH: ON" : "PRESS FLASH: OFF");
}

static void markChanged() {
  dirty = true;
  changedAt = millis();
  Serial.printf("[SETTINGS] pages=%s colours=%u,%u,%u,%u flash=%u\n",
                settings.multiPage ? "multi" : "single", settings.pageColour[0],
                settings.pageColour[1], settings.pageColour[2],
                settings.pageColour[3], settings.pressFlash);
}

static void save() {
  prefs.putBytes("settings", &settings, sizeof(settings));
  dirty = false;
}

static void onKeyOrButton(const isobus::VirtualTerminalClient::VTKeyEvent &event) {
  if (event.keyEvent !=
      isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
    return;

  if (event.objectID == KEY_SETTINGS) {
    vt->send_change_active_mask(WORKING_SET, SETTINGS_MASK);
  } else if (event.objectID == BTN_PAGE_MODE) {
    settings.multiPage = !settings.multiPage;
    sendPageMode();
    markChanged();
  } else if (event.objectID == BTN_RESTART) {
    save();
    Serial.println("[SETTINGS] restarting");
    restartAt = millis() + RESTART_DELAY_MS;
  } else if (event.objectID == BTN_PRESS_FLASH) {
    settings.pressFlash = !settings.pressFlash;
    sendPressFlash();
    markChanged();
  } else if (event.objectID >= BTN_PAGE_COLOUR_BASE &&
             event.objectID < BTN_PAGE_COLOUR_BASE + NUM_PAGES) {
    uint8_t page = event.objectID - BTN_PAGE_COLOUR_BASE;
    settings.pageColour[page] = (settings.pageColour[page] + 1) % NUM_PRESETS;
    sendPageColour(page);
    markChanged();
  }
}

void settings_load() {
  prefs.begin("settings", false);
  DeviceSettings stored;
  if (prefs.getBytes("settings", &stored, sizeof(stored)) == sizeof(stored)) {
    bool valid = stored.multiPage <= 1 && stored.pressFlash <= 1;
    for (uint8_t p = 0; p < NUM_PAGES; p++)
      valid &= stored.pageColour[p] < NUM_PRESETS;
    if (valid)
      settings = stored;
  }
  activeMultiPage = settings.multiPage;
}

bool settings_multi_page() { return activeMultiPage; }

uint32_t settings_page_colour(uint8_t page) {
  return PRESETS[settings.pageColour[page % NUM_PAGES]].rgb;
}

bool settings_press_flash() { return settings.pressFlash; }

void settings_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient) {
  vt = vtClient;
  vt->get_vt_soft_key_event_dispatcher().add_listener(onKeyOrButton);
  vt->get_vt_button_event_dispatcher().add_listener(onKeyOrButton);
}

void settings_update() {
  uint32_t now = millis();
  bool connected = vt->get_is_connected();
  if (connected && !vtWasConnected)
    vtSynced = false;
  vtWasConnected = connected;
  if (connected && !vtSynced) {
    bool ok = sendPageMode() && sendPressFlash();
    for (uint8_t p = 0; p < NUM_PAGES; p++)
      ok &= sendPageColour(p);
    vtSynced = ok; // retried next loop if the TX queue was full
  }

  if (dirty && now - changedAt >= SAVE_DELAY_MS)
    save();
  if (restartAt && (int32_t)(now - restartAt) >= 0)
    ESP.restart();
}

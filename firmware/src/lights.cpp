#include "lights.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Preferences.h>

#include "aux_buttons.h"
#include "object_pool/object_pool_ids.h"
#include "settings.h"

// Brain board header H7 (220 Ω series). SK6805 (handgrip) and most WS2811
// pixels take GRB order; 12 V WS2811 strips are often RGB instead.
#define LIGHTS_PIN 7
#define LIGHTS_COUNT 9
#define LIGHTS_ORDER NEO_GRB

static constexpr uint8_t STATUS_LEVEL = 64;          // red/green status at ~25%
static constexpr uint32_t VT_CONNECTED_GREEN_MS = 5000;
static constexpr uint32_t FRAME_MS = 20;             // 50 fps
static constexpr uint32_t RAINBOW_PERIOD_MS = 5000;  // one full hue turn
static constexpr uint32_t SAVE_DELAY_MS = 2000;      // batch NVS writes while the operator edits
static constexpr uint32_t PRESS_FLASH_MS = 200;

enum class LightMode : uint8_t { Rainbow = 0, Colour = 1, White = 2, Count = 3 };

// Padded to the length of the matching string in the object pool.
static const char *const MODE_TEXTS[] = {"MODE: RAINBOW", "MODE: COLOUR ",
                                         "MODE: WHITE  "};
static const char *const TOGGLE_TEXTS[] = {"OFF", "ON "};

struct LightSettings {
  uint8_t on;
  uint8_t mode;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t brightness; // percent, used by Rainbow and White
};

static Adafruit_NeoPixel strip(LIGHTS_COUNT, LIGHTS_PIN,
                               LIGHTS_ORDER + NEO_KHZ800);
static Preferences prefs;
static std::shared_ptr<isobus::VirtualTerminalClient> vt;

// Defaults match the initial values in the object pool.
static LightSettings settings = {0, (uint8_t)LightMode::Rainbow, 255, 255,
                                 255, 50};
static bool settingsDirty = false;
static uint32_t settingsChangedAt = 0;

static bool vtWasConnected = false;
static bool vtSynced = false;
static uint32_t vtConnectedAt = 0;
static uint32_t lastFrame = 0;
static uint32_t flashUntil = 0;

static void loadSettings() {
  LightSettings stored;
  if (prefs.getBytes("settings", &stored, sizeof(stored)) == sizeof(stored) &&
      stored.mode < (uint8_t)LightMode::Count && stored.brightness <= 100) {
    settings = stored;
  }
}

static void markChanged() {
  settingsDirty = true;
  settingsChangedAt = millis();
  Serial.printf("[LIGHTS] on=%u mode=%u rgb=%u,%u,%u brightness=%u%%\n",
                settings.on, settings.mode, settings.red, settings.green,
                settings.blue, settings.brightness);
}

// Push the current settings to the VT, so a (re)connected VT - which shows
// the pool's initial values - matches what the LEDs do.
static bool syncVt() {
  bool ok = true;
  ok &= vt->send_change_string_value(STR_KEY_TOGGLE_STATE,
                                     TOGGLE_TEXTS[settings.on ? 1 : 0]);
  ok &= vt->send_change_string_value(STR_LIGHTS_MODE,
                                     MODE_TEXTS[settings.mode]);
  ok &= vt->send_change_numeric_value(INPUT_RED, settings.red);
  ok &= vt->send_change_numeric_value(INPUT_GREEN, settings.green);
  ok &= vt->send_change_numeric_value(INPUT_BLUE, settings.blue);
  ok &= vt->send_change_numeric_value(INPUT_BRIGHTNESS, settings.brightness);
  return ok;
}

static void onKeyOrButton(const isobus::VirtualTerminalClient::VTKeyEvent &event) {
  if (event.keyEvent !=
      isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
    return;

  switch (event.objectID) {
  case KEY_LIGHTS_PAGE:
    vt->send_change_active_mask(WORKING_SET, LIGHTS_MASK);
    break;
  case KEY_BACK: // from the Lights and Settings masks
    vt->send_change_active_mask(WORKING_SET, MAIN_MASK);
    break;
  case KEY_LIGHTS_TOGGLE:
    settings.on = !settings.on;
    vt->send_change_string_value(STR_KEY_TOGGLE_STATE,
                                 TOGGLE_TEXTS[settings.on ? 1 : 0]);
    markChanged();
    break;
  case BTN_LIGHTS_MODE:
    settings.mode = (settings.mode + 1) % (uint8_t)LightMode::Count;
    vt->send_change_string_value(STR_LIGHTS_MODE, MODE_TEXTS[settings.mode]);
    markChanged();
    break;
  default:
    break;
  }
}

static void onNumericValue(
    const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event) {
  switch (event.objectID) {
  case INPUT_RED:
    settings.red = (uint8_t)min<uint32_t>(event.value, 255);
    break;
  case INPUT_GREEN:
    settings.green = (uint8_t)min<uint32_t>(event.value, 255);
    break;
  case INPUT_BLUE:
    settings.blue = (uint8_t)min<uint32_t>(event.value, 255);
    break;
  case INPUT_BRIGHTNESS:
    settings.brightness = (uint8_t)min<uint32_t>(event.value, 100);
    break;
  default:
    return;
  }
  markChanged();
}

static void render(uint32_t now) {
  bool connected = vt->get_is_connected();
  if (!connected) {
    strip.fill(strip.Color(STATUS_LEVEL, 0, 0));
  } else if (now - vtConnectedAt < VT_CONNECTED_GREEN_MS) {
    strip.fill(strip.Color(0, STATUS_LEVEL, 0));
  } else if (aux_buttons_multi_page()) {
    // The active page's colour, so the operator sees the page at a glance.
    uint32_t rgb = settings_page_colour(aux_buttons_current_page());
    uint8_t level = strip.gamma8(settings.brightness * 255 / 100);
    strip.fill(strip.Color(((rgb >> 16) & 0xFF) * level / 255,
                           ((rgb >> 8) & 0xFF) * level / 255,
                           (rgb & 0xFF) * level / 255));
  } else if (!settings.on) {
    strip.clear();
  } else {
    // Gamma-corrected so the brightness percentage looks even to the eye.
    uint8_t value = settings.brightness * 255 / 100;
    uint8_t level = strip.gamma8(value);
    switch ((LightMode)settings.mode) {
    case LightMode::Rainbow: {
      uint16_t hue0 = (uint32_t)(now % RAINBOW_PERIOD_MS) * 65536 /
                      RAINBOW_PERIOD_MS;
      for (uint16_t i = 0; i < LIGHTS_COUNT; i++) {
        uint16_t hue = hue0 + (uint32_t)i * 65536 / LIGHTS_COUNT;
        strip.setPixelColor(i,
                            strip.gamma32(strip.ColorHSV(hue, 255, value)));
      }
      break;
    }
    case LightMode::Colour:
      // R/G/B already set the brightness; the percentage isn't applied.
      strip.fill(strip.Color(settings.red, settings.green, settings.blue));
      break;
    case LightMode::White:
    default:
      strip.fill(strip.Color(level, level, level));
      break;
    }
  }

  // Press flash: invert what's showing - lit goes black, dark goes white.
  if ((int32_t)(flashUntil - now) > 0) {
    bool lit = false;
    for (uint16_t i = 0; i < LIGHTS_COUNT && !lit; i++)
      lit = strip.getPixelColor(i) != 0;
    if (lit)
      strip.clear();
    else
      strip.fill(strip.Color(255, 255, 255));
  }
  strip.show();
}

void lights_button_pressed() {
  if (settings_press_flash()) {
    flashUntil = millis() + PRESS_FLASH_MS;
    lastFrame = 0; // render now instead of at the next frame tick
  }
}

void lights_setup(std::shared_ptr<isobus::VirtualTerminalClient> vtClient) {
  vt = vtClient;
  prefs.begin("lights", false);
  loadSettings();

  strip.begin();
  strip.fill(strip.Color(STATUS_LEVEL, 0, 0)); // no VT yet
  strip.show();

  // Soft keys (page / back / on-off) and the on-mask MODE button share one
  // handler: both report the activated object ID.
  vt->get_vt_soft_key_event_dispatcher().add_listener(onKeyOrButton);
  vt->get_vt_button_event_dispatcher().add_listener(onKeyOrButton);
  vt->get_vt_change_numeric_value_event_dispatcher().add_listener(
      onNumericValue);
}

void lights_update() {
  uint32_t now = millis();

  bool connected = vt->get_is_connected();
  if (connected && !vtWasConnected) {
    vtConnectedAt = now;
    vtSynced = false;
    Serial.println("[LIGHTS] VT connected");
  } else if (!connected && vtWasConnected) {
    Serial.println("[LIGHTS] VT lost");
  }
  vtWasConnected = connected;
  if (connected && !vtSynced) {
    vtSynced = syncVt(); // retried next loop if the TX queue was full
  }

  if (settingsDirty && now - settingsChangedAt >= SAVE_DELAY_MS) {
    prefs.putBytes("settings", &settings, sizeof(settings));
    settingsDirty = false;
  }

  if (now - lastFrame >= FRAME_MS) {
    lastFrame = now;
    render(now);
  }
}

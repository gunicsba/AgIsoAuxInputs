#include "lights.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Preferences.h>
#include <cstring>

#include "driver/rmt.h"

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
// Unchanged frames are only resent this often (in case a pixel latched noise).
static constexpr uint32_t REFRESH_MS = 1000;

// The strip is driven by our own RMT channel rather than strip.show(): with
// ESP-IDF 4.4, Adafruit_NeoPixel installs the RMT driver anew for every frame
// with a single 48-bit memory block, so a 9-pixel frame (216 bits) needs
// several refill interrupts mid-frame. When one runs late (CAN or Wi-Fi
// interrupts, a flash write) the RMT runs dry and the pixels latch garbage -
// a visible blink. Here the whole frame fits the channel's memory, so the
// hardware clocks it out without any interrupt. Adafruit_NeoPixel is kept as
// the pixel buffer and for its colour helpers.
static constexpr rmt_channel_t LIGHTS_RMT_CHANNEL = RMT_CHANNEL_0;
static constexpr size_t LIGHTS_BITS = LIGHTS_COUNT * 3 * 8;
// 40 MHz RMT clock (80 MHz APB / 2): 25 ns per tick, 800 kHz WS2812 timing.
static constexpr rmt_item32_t LIGHTS_BIT_0 = {{{16, 1, 34, 0}}}; // 0.40 us high, 0.85 us low
static constexpr rmt_item32_t LIGHTS_BIT_1 = {{{32, 1, 18, 0}}}; // 0.80 us high, 0.45 us low
static rmt_item32_t rmtItems[LIGHTS_BITS];
static uint8_t sentPixels[LIGHTS_COUNT * 3];
static bool rmtReady = false;
static uint32_t lastSentAt = 0;

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

static void outputBegin() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)LIGHTS_PIN, LIGHTS_RMT_CHANNEL);
  config.clk_div = 2;
  // The frame plus the driver's end marker must fit, or it falls back to
  // interrupt refills (TX channels can borrow the following channels' blocks).
  config.mem_block_num =
      (LIGHTS_BITS + 1 + SOC_RMT_MEM_WORDS_PER_CHANNEL - 1) / SOC_RMT_MEM_WORDS_PER_CHANNEL;
  rmtReady = rmt_config(&config) == ESP_OK &&
             rmt_driver_install(LIGHTS_RMT_CHANNEL, 0, 0) == ESP_OK;
  if (!rmtReady)
    Serial.println("[LIGHTS] RMT setup failed, LEDs disabled");
}

// Sends the strip's pixel buffer if it changed (or REFRESH_MS passed).
static void outputShow(uint32_t now) {
  const uint8_t *pixels = strip.getPixels(); // already in the strip's colour order
  if (!rmtReady || (memcmp(pixels, sentPixels, sizeof(sentPixels)) == 0 &&
                    now - lastSentAt < REFRESH_MS))
    return;
  if (rmt_wait_tx_done(LIGHTS_RMT_CHANNEL, 0) != ESP_OK)
    return; // previous frame still going out; retried next frame
  size_t n = 0;
  for (size_t i = 0; i < sizeof(sentPixels); i++) {
    for (uint8_t mask = 0x80; mask; mask >>= 1)
      rmtItems[n++] = (pixels[i] & mask) ? LIGHTS_BIT_1 : LIGHTS_BIT_0;
  }
  rmt_write_items(LIGHTS_RMT_CHANNEL, rmtItems, n, false);
  memcpy(sentPixels, pixels, sizeof(sentPixels));
  lastSentAt = now;
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
  outputShow(now);
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

  outputBegin();
  strip.fill(strip.Color(STATUS_LEVEL, 0, 0)); // no VT yet
  outputShow(millis());

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

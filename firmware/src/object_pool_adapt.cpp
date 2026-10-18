#include "object_pool_adapt.h"

#include <Arduino.h>
#include <string.h>

#include "debug_log.h"

static constexpr uint8_t FONT_SIZE_COUNT = sizeof(AUX_POOL_FONT_SIZES) / sizeof(AUX_POOL_FONT_SIZES[0]);

const AuxPoolLayout &aux_pool_layout(bool multiPage, uint16_t maskSize) {
  return AUX_N_POOL_LAYOUTS[multiPage ? 1 : 0][maskSize < AUX_POOL_COMPACT_BELOW_MASK ? 1 : 0];
}

// The largest supported font that fits width x height (ISO 11783-6 4.6.16.3:
// best fit, never below 6x8).
static uint8_t bestFont(float width, float height, uint16_t supported) {
  constexpr float EPSILON = 0.001f;
  for (int size = FONT_SIZE_COUNT - 1; size > 0; size--) {
    if ((supported & (1u << size)) && AUX_POOL_FONT_SIZES[size][0] <= width + EPSILON &&
        AUX_POOL_FONT_SIZES[size][1] <= height + EPSILON)
      return size;
  }
  return 0;
}

static void put16(uint8_t *at, int value) {
  at[0] = value & 0xFF;
  at[1] = (value >> 8) & 0xFF;
}

static int scaledSize(int value, float scale) {
  return max(1, (int)(value * scale));
}

static void scale(uint8_t *buffer, const AuxPoolLayout &layout, const VtDisplay &vt) {
  const float mask = (float)vt.maskSize / layout.maskSize;
  // Square keys (at least 3/4 as high as wide) get the square variant
  bool square = false;
  float keyX = 1, keyY = 1;
  if (vt.keyWidth != 0 && vt.keyHeight != 0) {
    square = vt.keyHeight * 4 >= vt.keyWidth * 3;
    keyX = (float)vt.keyWidth / (square ? AUX_POOL_KEY_SQUARE_W : AUX_POOL_KEY_WIDE_W);
    keyY = (float)vt.keyHeight / (square ? AUX_POOL_KEY_SQUARE_H : AUX_POOL_KEY_WIDE_H);
  }

  for (uint32_t i = 0; i < layout.scaleCount; i++) {
    const AuxPoolScaleField &field = layout.scale[i];
    uint8_t *at = &buffer[field.offset];
    const int value = square ? field.square : field.wide;
    switch (field.kind) {
      case AUX_SCALE_MASK_POS: put16(at, (int)(value * mask)); break;
      case AUX_SCALE_MASK_SIZE: put16(at, scaledSize(value, mask)); break;
      case AUX_SCALE_MASK_LINE: *at = scaledSize(value, mask); break;
      case AUX_SCALE_MASK_FONT:
        *at = bestFont(AUX_POOL_FONT_SIZES[value][0] * mask, AUX_POOL_FONT_SIZES[value][1] * mask,
                       vt.fontsSupported);
        break;
      case AUX_SCALE_KEY_X: put16(at, (int)(value * keyX)); break;
      case AUX_SCALE_KEY_Y: put16(at, (int)(value * keyY)); break;
      case AUX_SCALE_KEY_W: put16(at, scaledSize(value, keyX)); break;
      case AUX_SCALE_KEY_H: put16(at, scaledSize(value, keyY)); break;
      case AUX_SCALE_KEY_FONT:
        *at = bestFont(AUX_POOL_FONT_SIZES[value][0] * keyX, AUX_POOL_FONT_SIZES[value][1] * keyY,
                       vt.fontsSupported);
        break;
      default: break;
    }
  }
  debug_log("[AUX-N] %s layout scaled x%.2f to the %u data mask, %s soft keys x%.2f/x%.2f",
            layout.name, mask, (unsigned)vt.maskSize, square ? "square" : "wide", keyX, keyY);
}

void aux_pool_adapt(uint8_t *buffer, const AuxPoolLayout &layout, const VtDisplay &vt) {
  memcpy(buffer, layout.data, layout.size);
  scale(buffer, layout, vt);

  if (vt.versionByte >= 4 && vt.versionByte != 0xFF) {
    // Vertical justification bits are reserved on VT3 and older, and a
    // VT3/Mueller unit rejected the pool outright with them set (see
    // JUSTIFY_HORIZONTAL_MASK in tools/build_aux_pool.py): patch the full
    // justification back in.
    for (uint32_t i = 0; i < layout.justifyCount; i++) {
      buffer[layout.justify[i].offset] = layout.justify[i].value;
    }
    debug_log("[AUX-N] VT version 4+: upgraded justification");
  } else if (vt.versionByte < 3 || vt.versionByte == 0xFF) {
    // AUX-N objects don't exist before VT3, and an AMATRON 3 (version byte
    // 2) rejected the pool at the first one. Blank them out into String
    // Variables of the same length (see VT3_OBJECT_TYPES in the builder).
    // The pool still loads, but the VT can't assign the inputs.
    for (uint32_t i = 0; i < layout.vt3ObjectCount; i++) {
      uint8_t *object = &buffer[layout.vt3Objects[i].offset];
      uint16_t valueLength = layout.vt3Objects[i].length - AUX_POOL_STRING_VARIABLE_HEADER;
      object[2] = AUX_POOL_TYPE_STRING_VARIABLE; // keeps its object ID
      object[3] = valueLength & 0xFF;
      object[4] = valueLength >> 8;
      memset(&object[AUX_POOL_STRING_VARIABLE_HEADER], ' ', valueLength);
    }
    debug_log("[AUX-N] VT2 or older: AUX-N not supported, blanked %lu AUX-N objects",
              (unsigned long)layout.vt3ObjectCount);
  }
}

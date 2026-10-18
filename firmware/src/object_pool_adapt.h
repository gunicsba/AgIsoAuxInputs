/*
 * Adapts the built-in object pools to the connected VT before the upload.
 *
 * ISO 11783-6 leaves it to the Working Set to fit its pool to the VT: scale
 * positions, sizes and fonts to the VT's Data Mask and Soft Key designator,
 * and only use objects and attributes the VT's version knows. The pools are
 * built (tools/build_aux_pool.py) for a reference size with every field
 * that depends on it recorded; this rewrites those fields in place, so the
 * pool keeps its size and object layout.
 */
#pragma once

#include <stdint.h>

#include "object_pool/aux_n_pool_data.h"

struct VtDisplay {
  int versionByte;        // raw VT version from the Get Memory response
  uint16_t maskSize;      // Data Mask width = height, px
  uint8_t keyWidth;       // Soft Key designator, px
  uint8_t keyHeight;
  uint16_t fontsSupported; // bit n = ISO font size n (6x8 = bit 0) is supported
};

// The full layout, or the compact one for a data mask below
// AUX_POOL_COMPACT_BELOW_MASK. Both layouts of a mode have the same size.
const AuxPoolLayout &aux_pool_layout(bool multiPage, uint16_t maskSize);

// Copies the layout into buffer and adapts it to the VT: scaled to its data
// mask and soft key designator, then VT4 justification or VT2 AUX-N
// blanking. Logs what it did.
void aux_pool_adapt(uint8_t *buffer, const AuxPoolLayout &layout, const VtDisplay &vt);

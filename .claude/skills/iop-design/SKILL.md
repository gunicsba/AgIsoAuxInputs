---
name: iop-design
description: Rules for designing or changing ISO 11783-6 (ISOBUS VT) object pools - .iop files, pool builder scripts, AUX-N inputs, data/soft key masks - so they load and look right on real terminals, not just AgIsoVirtualTerminal. Use before adding or changing any VT object, layout, font, designator or soft key, when a VT rejects a pool ("Error in end of object pool message", "Faulty Object"), or when a pool looks clipped or tiny on some terminal.
---

# Designing VT object pools that work on real terminals

AgIsoVirtualTerminal accepts and renders almost anything. Real terminals reject
the whole pool over one unsupported attribute, and come in very different sizes.
Each rule below cost a field test to discover. Work through the checklist at the
end before you consider a pool change done.

## Know the range you're designing for

The pool must work on every terminal it may meet, not only the one at hand.
These were all seen in the field:

| VT | VT version | Data mask | Soft key designator | Touch |
|---|---|---|---|---|
| AgIsoVirtualTerminal | 4 | 480x480 | 60x60 | yes |
| Mueller BASIC Terminal | 3 | 400x400 | 120x80 | no |
| Amazone AMATRON 3 | 2 | 200x200 | 60x47 | no |

ISO 11783-6 limits: data masks are square, any size from 200x200 (480x480 from
VT version 6); soft key designators are at least 60x32 (60x60 from VT6). The VT
reports its version (Get Memory response), soft key size (Get Number of Soft
Keys), fonts (Get Text Font Data) and mask size and touch/pointing support (Get
Hardware) before the pool is uploaded.

## Rules

### 1. Stay within the VT's version - one unknown thing rejects everything
- A VT that meets an object type or attribute value its version doesn't define
  fails the whole pool, usually naming the first faulty object.
  - AUX-N objects (Auxiliary Function Type 2, Auxiliary Input Type 2, Auxiliary
    Control Designator Type 2 Object Pointer) need VT3. A VT2 AMATRON 3 rejected
    the pool at the first Auxiliary Input Type 2.
  - Vertical justification bits (Output String / Input Number justification
    bits 2-3) are reserved before VT4. A VT3 Mueller rejected the pool with them
    set.
  - Key Group, Window Mask and other later objects need VT4+; Colour Palette,
    Graphic Data, Scaled Graphic and Working Set Special Controls need VT6.
- Ship the pool for the lowest version you support, and upgrade or downgrade at
  runtime once the version is known. The pool size has already been sent by
  then, so patch in place without changing lengths: write the newer attribute
  value, or turn an unsupported object into a same-length String Variable
  (ID, type 22, length, spaces) that nothing references.
- AgIsoStack's `get_connected_vt_version()` reports raw version bytes 0-2 as
  "unknown". Read the raw byte from the Get Memory response yourself.

### 2. Everything in a soft key designator must fit the VT's designator
- Key children, the Working Set designator and Auxiliary Input designators are
  all drawn in a soft key designator. 60x60 designators contributed to the
  AMATRON 3 rejecting the pool; its designator is 60x47.
- Ship them sized for 60x32, the ISO minimum, and scale up to the reported size
  at runtime. Square-ish (60x60, 60x47) and wide (120x80, 60x32) designators want
  different arrangements: plan both.

### 3. Scaling is the Working Set's job (ISO 11783-6:2014 4.6.16)
- The VT does not scale your masks. The Working Set adapts positions, sizes and
  fonts to the reported mask and designator sizes, before or after upload.
- Fonts: choose the largest font the VT supports that fits the scaled size;
  6x8 is the floor. Text clips once scaling drives everything to 6x8, so a 480
  layout scaled to 200 is unreadable. Hand-design a second layout for small
  masks, and scale each layout to the exact size.
- Picture Graphic bitmaps are the exception: the VT scales them to the object's
  width attribute. Patch that one field; never resample bitmaps. Store images at
  the largest size you need - shrinking looks better than enlarging.

### 4. Every text must fit its box
- For non-proportional fonts: characters x font width <= box width, and font
  height <= box height. Check this at every size you scale to.
- A string the firmware changes at runtime has the pool string's length as its
  capacity: put the longest variant (or padding) in the pool and pad every
  update to that length.

### 5. Centre text with boxes on VT3 and older
- Without vertical justification, text sits at the top of its box. To centre it
  vertically, make the box one font high and position it centred, instead of
  making it full height.

### 6. Expect no touchscreen
- Both field terminals above have none (Get Hardware features bits 0-1 clear).
  On-screen Buttons and input fields then work only through the VT's navigation
  and confirm keys, and operators read them as broken. Put essential actions on
  soft keys; treat on-screen Buttons as extras.

### 7. Version labels must follow the content
- VTs cache pools by version label per Working Set NAME and skip the upload if
  the label matches. Derive the label from the pool bytes as actually uploaded
  (after runtime patching), never from a hard-coded string, or terminals keep
  serving a stale pool.

### 8. AUX-N inputs (ISO 11783-6:2014 Annex J)
- Keep input object IDs stable across firmware and modes. VTs restore saved
  assignments by NAME + input object ID, and AgIsoVirtualTerminal hangs when a
  restored assignment points at an input that no longer exists. Give input sets
  that differ their own NAME (identity number) and model identification code.
- The VT allows no manual assignment to an implement's functions until that
  implement sent a valid Preferred Assignment (J.5, J.6). An implement that
  prefers your input unit waits for your Auxiliary Input Type 2 Maintenance
  message with status Ready (J.7.7, J.7.10: every 100 ms, global, Ready only
  after a successful End of Object Pool or Load Version response). A broken
  maintenance message therefore blocks assignment for the implement too.
- Declare the function type the hardware really is (Table J.5): the VT only
  pairs an input with a function of exactly the same type. A spring-centred
  joystick axis is type 3 ("return to 50%"), not type 1 ("maintains
  position").
- Status messages (J.7.9): none for an input until the VT enabled it (except
  in learn mode). Object ID FFFF in a Status Enable command means all inputs,
  disable only (J.7.11) - answering it with an error can make the VT drop the
  input unit. Analog values are 0..0xFAFF; anything above is reserved, error
  or not available. A momentary button reports 1 when pressed, then 2 while
  held, repeated every 200 ms: an implement treats a held input as released
  after 300 ms without one. Value 2 counts presses (off to not off) only.
- AgIsoStack's VT client gets the status rules wrong; this repository patches
  it at build time (`firmware/tools/patch_agisostack.py`). Recheck the patch
  whenever the pinned AgIsoStack commit moves.

## Reading a rejection

The End of Object Pool response carries the first faulty object ID and its
parent, plus two bitfields:
- Error codes: bit 0 = error in the object pool, bit 1 = VT ran out of memory,
  bit 4 = any other error.
- Object pool error codes: bit 0 = method or attribute not supported by the VT,
  bit 1 = unknown object reference, bit 2 = any other error, bit 3 = pool deleted
  from volatile memory.

"Other error" on an object usually means an out-of-range size or a value the VT's
version doesn't define (rules 1 and 2), not a byte-layout bug. Fix the cause for
every object of that kind: the VT only reports the first one.

## Checklist

1. Lowest VT version supported, and nothing in the shipped pool above it
   (rule 1).
2. Designator content fits 60x32 (rule 2).
3. The layouts fit 200 and 480 masks, and text still fits after scaling (rules
   3 and 4).
4. Runtime strings padded to their pool length (rule 4).
5. Essential actions on soft keys (rule 6).
6. The version label is derived from the uploaded bytes (rule 7).
7. Tested on at least one real VT3 or older terminal, not just
   AgIsoVirtualTerminal.

## In this repository

- `firmware/tools/build_aux_pool.py` builds the pools: two page modes, each with
  a full (480) and a compact (200) layout padded to the same size. Each pool
  records its scalable fields (data mask fields and square/wide soft key
  fields), its VT4 justification upgrades and its VT3-only objects. Every text
  and mask child is checked at the reference size.
- `firmware/src/object_pool_adapt.cpp` picks the layout, scales it to the VT's
  mask and soft keys with best-fit supported fonts, and applies the VT2/VT4
  patches before the upload.
- After changing a layout run `python tools/build_aux_pool.py`, then
  `python tools/check_pool_scaling.py` (from `firmware/`). The check scales the
  pools like the firmware for several terminals and checks everything fits; add
  a profile when you meet a new terminal. Keep its `scale()` in step with
  `object_pool_adapt.cpp`.
- `/debug` on the board logs the VT's version byte, soft key and mask sizes,
  hardware features and operator input events.

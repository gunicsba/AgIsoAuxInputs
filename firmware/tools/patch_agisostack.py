"""
PlatformIO pre-build script: patches AgIsoStack's VT client (pinned in
platformio.ini) where its auxiliary input unit side doesn't follow
ISO 11783-6:2014 Annex J. Still needed on AgIsoStack main as of 2026-10.

1. J.7.9: an input unit shall not send an Auxiliary Input Type 2 Status
   message for an input until the VT enabled it (except in learn mode).
   AgIsoStack sent every input's status once a second regardless.
2. J.7.11: object ID FFFF in an Auxiliary Input Status Enable command means
   all inputs (disable only). AgIsoStack answered it "invalid object ID",
   which a VT may take as the input unit not being available.
3. J.7.9: a held non-latched Boolean (value 1 = 2) repeats its status every
   200 ms; an implement treats it as released after 300 ms without one.
   AgIsoStack only sent changes (and once a second).

Each patch is applied once per library copy (every env has its own under
.pio/libdeps/<env>/); a patched copy is recognised by the new code. The build fails if the code to
patch isn't there, e.g. after moving the pinned commit - check whether the
fix landed upstream and drop or update the patch.
"""
import pathlib

Import("env")  # noqa: F821 - provided by PlatformIO

PATCHES = [
    (
        "J.7.9 status only when enabled",
        """		bool retVal = false;
		AuxiliaryInputState &state = ourAuxiliaryInputs.at(objectID);
		/// @todo Change status message every 50ms to every 200ms for non-latched boolean inputs on interaction
		if (SystemTiming::time_expired_ms(state.lastStatusUpdate, AUXILIARY_INPUT_STATUS_DELAY) ||""",
        """		bool retVal = false;
		AuxiliaryInputState &state = ourAuxiliaryInputs.at(objectID);
		// AgIsoAuxInputs patch: ISO 11783-6 J.7.9 - no status until the VT enabled this input, except in learn mode
		if ((!state.enabled) && (!get_auxiliary_input_learn_mode_enabled()))
		{
			return false;
		}
		if (SystemTiming::time_expired_ms(state.lastStatusUpdate, AUXILIARY_INPUT_STATUS_DELAY) ||""",
    ),
    (
        "J.7.9 held repeat",
        """			if ((value1 != ourAuxiliaryInputs.at(auxiliaryInputID).value1) || (value2 != ourAuxiliaryInputs.at(auxiliaryInputID).value2))
			{""",
        """			// AgIsoAuxInputs patch: ISO 11783-6 J.7.9 - a held non-latched Boolean (value 1 = 2) repeats every 200 ms
			constexpr std::uint32_t AUXILIARY_INPUT_HELD_REPEAT_MS = 200;
			const AuxiliaryInputState &current = ourAuxiliaryInputs.at(auxiliaryInputID);
			const bool heldRepeat = (2 == value1) && (2 == current.value1) &&
			  SystemTiming::time_expired_ms(static_cast<std::uint32_t>(current.lastStatusUpdate), AUXILIARY_INPUT_HELD_REPEAT_MS);
			if ((value1 != current.value1) || (value2 != current.value2) || heldRepeat)
			{""",
    ),
    (
        "J.7.11 disable all",
        """							std::uint16_t inputObjectID = message.get_uint16_at(1);
							bool shouldEnable = message.get_bool_at(3, 0);
""",
        """							std::uint16_t inputObjectID = message.get_uint16_at(1);
							bool shouldEnable = message.get_bool_at(3, 0);
							// AgIsoAuxInputs patch: ISO 11783-6 J.7.11 - FFFF addresses all inputs, and only to disable them
							if (NULL_OBJECT_ID == inputObjectID)
							{
								if (!shouldEnable)
								{
									for (auto &input : parentVT->ourAuxiliaryInputs)
									{
										input.second.enabled = false;
									}
								}
								parentVT->send_auxiliary_input_status_enable_response(inputObjectID, false, shouldEnable);
								break;
							}
""",
    ),
    (
        "J.7.9 status right after enabling",
        """							if (!isInvalidObjectID)
							{
								result->second.enabled = shouldEnable;
							}""",
        """							if (!isInvalidObjectID)
							{
								// AgIsoAuxInputs patch: send the input's current status right after it's enabled
								if (shouldEnable && !result->second.enabled)
								{
									result->second.lastStatusUpdate = 0;
								}
								result->second.enabled = shouldEnable;
							}""",
    ),
]


def patch_library():
    libdeps = pathlib.Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")  # noqa: F821
    sources = list(libdeps.glob("*/isobus/src/isobus_virtual_terminal_client.cpp"))
    if not sources:
        raise SystemExit(f"patch_agisostack: AgIsoStack not found under {libdeps}")
    for source in sources:
        text = source.read_text(encoding="utf-8")
        applied = []
        for name, old, new in PATCHES:
            if new in text:
                continue
            if text.count(old) != 1:
                raise SystemExit(f"patch_agisostack: '{name}' doesn't match {source} "
                                 "- did the pinned AgIsoStack commit change?")
            text = text.replace(old, new)
            applied.append(name)
        if applied:
            source.write_text(text, encoding="utf-8")
            print(f"patch_agisostack: {', '.join(applied)} -> {source}")


patch_library()

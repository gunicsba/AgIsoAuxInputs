"""
PlatformIO pre-build script: patches AgIsoStack's VT client (pinned in
platformio.ini) where its auxiliary input unit side doesn't follow
ISO 11783-6:2014 Annex J. Still needed on AgIsoStack main as of 2026-10-30
(ee6eba1); the J.7.9 "no status until enabled" and J.7.11 "FFFF disables
all inputs" patches this script also carried are upstream since #655.

1. J.7.9: a held non-latched Boolean (value 1 = 2) repeats its status every
   200 ms; an implement treats it as released after 300 ms without one.
   AgIsoStack only sent changes (and once a second).
2. J.7.9: an input's status goes out right after the VT enables it, not up
   to a second later.
3. Object pools uploaded in several parts: a failed part started the next
   part's transfer, whose result was then lost, and the upload never
   started again.
4. An offline VT partner takes the next matching VT (upstream #870).

Each patch is applied once per library copy (every env has its own under
.pio/libdeps/<env>/); a patched copy is recognised by the new code. The build fails if the code to
patch isn't there, e.g. after moving the pinned commit - check whether the
fix landed upstream and drop or update the patch.
"""
import pathlib

Import("env")  # noqa: F821 - provided by PlatformIO

VT_CLIENT = "isobus_virtual_terminal_client.cpp"
NETWORK_MANAGER = "can_network_manager.cpp"

PATCHES = [
    (
        VT_CLIENT,
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
        VT_CLIENT,
        "J.7.9 status right after enabling",
        """								if (!isInvalidObjectID)
								{
									result->second.enabled = shouldEnable;
								}""",
        """								if (!isInvalidObjectID)
								{
									// AgIsoAuxInputs patch: send the input's current status right after it's enabled
									if (shouldEnable && !result->second.enabled)
									{
										result->second.lastStatusUpdate = 0;
									}
									result->second.enabled = shouldEnable;
								}""",
    ),
    # Multi-part object pools (several set_object_pool() indexes): after a
    # part failed, the upload loop went on and started the next part while
    # resetting the connection. That session's result came outside
    # UploadObjectPool and was dropped, so the upload state stayed
    # "in progress" and every later connection waited for it forever.
    (
        VT_CLIENT,
        "multi-part upload: stop after a failed part",
        "\t\t\t\t\t\t\t\tLOG_ERROR(\"[VT]: An object pool failed to upload. Resetting connection to VT.\");\n"
        "\t\t\t\t\t\t\t\tset_state(StateMachineState::Disconnected);\n",
        "\t\t\t\t\t\t\t\tLOG_ERROR(\"[VT]: An object pool failed to upload. Resetting connection to VT.\");\n"
        "\t\t\t\t\t\t\t\tset_state(StateMachineState::Disconnected);\n"
        "\t\t\t\t\t\t\t\t// AgIsoAuxInputs patch: don't start the next part's transfer\n"
        "\t\t\t\t\t\t\t\tallPoolsProcessed = false;\n"
        "\t\t\t\t\t\t\t\tbreak;\n",
    ),
    (
        VT_CLIENT,
        "multi-part upload: no stale transfer state",
        "\t\t\t\t\tif (firstTimeInState)\n"
        "\t\t\t\t\t{\n"
        "\t\t\t\t\t\tif (get_any_pool_needs_scaling())\n",
        "\t\t\t\t\tif (firstTimeInState)\n"
        "\t\t\t\t\t{\n"
        "\t\t\t\t\t\t// AgIsoAuxInputs patch: a transfer of an earlier connection can't still be running\n"
        "\t\t\t\t\t\tcurrentObjectPoolState = CurrentObjectPoolUploadState::Uninitialized;\n"
        "\t\t\t\t\t\tif (get_any_pool_needs_scaling())\n",
    ),
    (
        NETWORK_MANAGER,
        "offline partner takes the next matching NAME",
        "\t\t\tif (nullptr == foundControlFunction)\n"
        "\t\t\t{\n"
        "\t\t\t\t// If we still haven't found it, it might be a partner. Check the list of partners.\n"
        "\t\t\t\tfor (const auto &partner : partneredControlFunctions)\n"
        "\t\t\t\t{\n"
        "\t\t\t\t\tif ((partner->get_can_port() == rxFrame.channel) &&\n"
        "\t\t\t\t\t    (partner->check_matches_name(NAME(claimedNAME))) &&\n"
        "\t\t\t\t\t    (0 == partner->get_NAME().get_full_name()))\n"
        "\t\t\t\t\t{\n",
        "\t\t\t// AgIsoAuxInputs patch: an offline partner takes the next matching NAME\n"
        "\t\t\tif ((nullptr == foundControlFunction) || (ControlFunction::Type::External == foundControlFunction->get_type()))\n"
        "\t\t\t{\n"
        "\t\t\t\t// If we still haven't found it, it might be a partner. Check the list of partners.\n"
        "\t\t\t\tfor (const auto &partner : partneredControlFunctions)\n"
        "\t\t\t\t{\n"
        "\t\t\t\t\tif ((partner->get_can_port() == rxFrame.channel) &&\n"
        "\t\t\t\t\t    (partner->check_matches_name(NAME(claimedNAME))) &&\n"
        "\t\t\t\t\t    ((0 == partner->get_NAME().get_full_name()) || !partner->get_address_valid()))\n"
        "\t\t\t\t\t{\n"
        "\t\t\t\t\t\tif (nullptr != foundControlFunction)\n"
        "\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\tinactiveControlFunctions.remove(foundControlFunction);\n"
        "\t\t\t\t\t\t\tfor (auto &entry : controlFunctionTable[rxFrame.channel])\n"
        "\t\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\t\tif (entry == foundControlFunction)\n"
        "\t\t\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\t\t\tentry = nullptr;\n"
        "\t\t\t\t\t\t\t\t}\n"
        "\t\t\t\t\t\t\t}\n"
        "\t\t\t\t\t\t\tfoundControlFunction->address = NULL_CAN_ADDRESS;\n"
        "\t\t\t\t\t\t}\n"
        "\t\t\t\t\t\tif (0 != partner->get_NAME().get_full_name())\n"
        "\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\tLOG_INFO(\"[NM]: Offline partner %016llx replaced by %016llx on channel %u.\",\n"
        "\t\t\t\t\t\t\t         partner->get_NAME().get_full_name(),\n"
        "\t\t\t\t\t\t\t         claimedNAME,\n"
        "\t\t\t\t\t\t\t         rxFrame.channel);\n"
        "\t\t\t\t\t\t}\n",
    ),
]


def patch_library():
    libdeps = pathlib.Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")  # noqa: F821
    sources = [source for file_name in sorted({patch[0] for patch in PATCHES})
               for source in libdeps.glob(f"*/isobus/src/{file_name}")]
    if not sources:
        raise SystemExit(f"patch_agisostack: AgIsoStack not found under {libdeps}")
    for source in sources:
        text = source.read_text(encoding="utf-8")
        applied = []
        for file_name, name, old, new in PATCHES:
            if file_name != source.name:
                continue
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

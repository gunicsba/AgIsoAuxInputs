#if defined(ENABLE_TRITON_POOL)
#include "triton_ws.h"

#include "debug_log.h"
#include "isobus/VTObjectPool.hpp" // Triton's object IDs (custom_triton_dir)
#include "isobus/isobus/can_NAME.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "object_pool/triton_pool.h"

namespace triton_ws {

using isobus::VirtualTerminalClient;
using Activation = VirtualTerminalClient::KeyActivationCode;

static std::shared_ptr<isobus::InternalControlFunction> ecu;
static std::shared_ptr<isobus::PartneredControlFunction> partner;
static std::shared_ptr<VirtualTerminalClient> vt;

static uint32_t offsetCm = 0; // what Wider/Narrower step (Var_Offset)
static uint32_t keyEvents = 0, auxEvents = 0;
static bool auxHeld[2] = {false, false}; // Aux_Wider, Aux_Narrower
static bool wasConnected = false;

static void stepOffset(bool wider, const char *source) {
  if (wider && offsetCm < 9999)
    offsetCm++;
  else if (!wider && offsetCm > 0)
    offsetCm--;
  vt->send_change_numeric_value(triton::Var_Offset, offsetCm);
  debug_log("[TRITON] %s %s: offset %lu cm", source, wider ? "Breder" : "Smaller",
            (unsigned long)offsetCm);
}

static void showMask(uint16_t dataMask) {
  vt->send_change_active_mask(triton::Plough_WorkingSet, dataMask);
}

static void onKey(const VirtualTerminalClient::VTKeyEvent &event) {
  keyEvents++;
  if (event.keyEvent == Activation::ButtonUnlatchedOrReleased ||
      event.keyEvent == Activation::ButtonPressAborted) {
    debug_log("[TRITON] key %u %s", event.objectID,
              event.keyEvent == Activation::ButtonPressAborted ? "aborted" : "released");
    return;
  }
  if (event.keyEvent != Activation::ButtonPressedOrLatched)
    return; // held repeats
  debug_log("[TRITON] key %u pressed", event.objectID);
  switch (event.objectID) {
    case triton::Key_Wider: stepOffset(true, "soft key"); break;
    case triton::Key_Narrower: stepOffset(false, "soft key"); break;
    // The pages, as the plough opens them (nothing is saved here)
    case triton::Key_Calibrate: showMask(triton::Plough_CalDataMask); break;
    case triton::Key_CalSave:
    case triton::Key_CalCancel: showMask(triton::Plough_DataMask); break;
    case triton::Key_CalWidth: showMask(triton::Plough_CalWidthMask); break;
    case triton::Key_CalWidthBack: showMask(triton::Plough_CalDataMask); break;
    default: break;
  }
}

// Boolean momentary: value 1 is 0 released, 1 pressed, 2 held
static void onAuxFunction(const VirtualTerminalClient::AuxiliaryFunctionEvent &event) {
  const uint16_t function = event.function.functionObjectID;
  if (function != triton::Aux_Wider && function != triton::Aux_Narrower)
    return;
  auxEvents++;
  const bool wider = function == triton::Aux_Wider;
  const bool held = event.value1 != 0;
  bool &wasHeld = auxHeld[wider ? 0 : 1];
  if (held == wasHeld)
    return;
  wasHeld = held;
  debug_log("[TRITON] AUX %s %s (input %u, value %u)", wider ? "Breder" : "Smaller",
            held ? "pressed" : "released", event.function.inputObjectID, event.value1);
  if (held)
    stepOffset(wider, "AUX");
}

void setup(std::shared_ptr<isobus::PartneredControlFunction> vtPartner) {
  partner = vtPartner;
  // Triton's own NAME fields (IsobusGuidanceChannel.cpp: tillage machine
  // control) with our manufacturer code and identity number, so it can't
  // take a real Triton's stored pool or AUX assignments
  isobus::NAME name(0);
  name.set_arbitrary_address_capable(true);
  name.set_industry_group(2);
  name.set_device_class(2);
  name.set_function_code(static_cast<uint8_t>(isobus::NAME::Function::MachineControl));
  name.set_identity_number(3);
  name.set_ecu_instance(0);
  name.set_function_instance(0);
  name.set_device_class_instance(0);
  name.set_manufacturer_code(1407);
  ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(name, 0);

  vt = std::make_shared<VirtualTerminalClient>(partner, ecu);
  vt->set_object_pool(0, TRITON_POOL, sizeof(TRITON_POOL), TRITON_POOL_VERSION);
  // Drawn for a 200 px data mask and 60 px soft keys, scaled by AgIsoStack
  // as Triton does (IsobusVtInterface.cpp)
  vt->set_object_pool_scaling(0, 200, 60);
  vt->get_vt_soft_key_event_dispatcher().add_listener(onKey);
  vt->get_vt_button_event_dispatcher().add_listener(onKey);
  vt->set_auxiliary_functions_enabled(true);
  vt->get_auxiliary_function_event_dispatcher().add_listener(onAuxFunction);
  vt->initialize(false);
  debug_log("[TRITON] started: pool %u bytes, version %s", (unsigned)sizeof(TRITON_POOL),
            TRITON_POOL_VERSION);
}

void update() {
  vt->update();
  const bool connected = vt->get_is_connected();
  if (connected && !wasConnected) // the pool starts at 0
    vt->send_change_numeric_value(triton::Var_Offset, offsetCm);
  wasConnected = connected;
}

void debug_status(String &out) {
  out += "Triton plough pool (third working set)\n  address ";
  out += ecu->get_address_valid() ? String(ecu->get_address()) : "not claimed";
  out += ", VT " + String(vt->get_is_connected() ? "connected" : (partner->get_address_valid() ? "found" : "not found")) +
         " (" + String(vt->get_number_x_pixels()) + "x" + String(vt->get_number_y_pixels()) +
         ", soft key " + String(vt->get_softkey_x_axis_pixels()) + "x" +
         String(vt->get_softkey_y_axis_pixels()) + ")\n  pool " + String(sizeof(TRITON_POOL)) +
         " bytes, version " TRITON_POOL_VERSION ", offset " + String(offsetCm) + " cm, " +
         String(keyEvents) + " key events, " + String(auxEvents) + " AUX events\n";
}

} // namespace triton_ws
#endif

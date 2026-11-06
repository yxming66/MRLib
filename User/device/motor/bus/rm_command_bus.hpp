#pragma once
#include "device/motor/bus/can_transport.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/protocol/rm_command_group.hpp"
namespace mr::motor {
// One app-owned instance per physical port. All calls run in one control task.
// BeginCycle -> stage every intended motor -> Flush (including error paths).
// Unstaged slots are zero. A failed group is retryable until the next cycle.
class RmCommandBus final {
 public:
  explicit RmCommandBus(CanPort& port) : transport_(&port) {}
  RmCommandBus(const RmCommandBus&) = delete;
  RmCommandBus& operator=(const RmCommandBus&) = delete;
  CanPort* port() const { return transport_.port(); }
  bool CanRegister(const RmCommandAddress& address) const {
    return Valid(address) && !slots_[Index(address)].owner;
  }
  int8_t Register(const RmCommandAddress& address, const void* owner, MotorState& state) {
    if (!owner || !CanRegister(address)) return DEVICE_ERR;
    auto& slot = slots_[Index(address)];
    slot.owner = owner; slot.state = &state;
    return DEVICE_OK;
  }
  void Release(const void* owner) {
    for (auto& slot : slots_) if (slot.owner == owner) slot = {};
  }
  void BeginCycle() {
    open_ = true; sealed_ = false;
    for (auto& done : sent_) done = false;
    for (auto& slot : slots_) {
      slot.raw = 0; slot.staged = false; slot.relax = true;
      if (slot.state) slot.state->command_pending = false;
    }
  }
  int8_t Stage(const void* owner, int16_t raw, bool relax) {
    if (!open_ || sealed_) return DEVICE_ERR;
    for (auto& slot : slots_) if (slot.owner == owner) {
      slot.raw = static_cast<uint16_t>(raw); slot.staged = true; slot.relax = relax;
      slot.state->command_pending = true; slot.state->last_commit_ok = false;
      return DEVICE_OK;
    }
    return DEVICE_ERR_NO_DEV;
  }
  bool Pending(const void* owner) const {
    for (const auto& slot : slots_)
      if (slot.owner == owner) return slot.state->command_pending;
    return false;
  }
  void Cancel(const void* owner) {
    for (auto& slot : slots_) if (slot.owner == owner) {
      slot.raw = 0; slot.staged = false; slot.relax = true;
      slot.state->command_pending = false;
    }
  }
  int8_t Flush() {
    if (!open_) return DEVICE_OK;
    sealed_ = true;
    int8_t result = DEVICE_OK;
    for (size_t group = 0; group < 3; ++group) {
      if (sent_[group]) continue;
      bool used = false;
      CanFrame frame{}; frame.id = group == 0 ? 0x200 : (group == 1 ? 0x1ff : 0x2ff);
      frame.dlc = 8;
      for (size_t i = 0; i < 4; ++i) {
        const auto& slot = slots_[group * 4 + i];
        used = used || slot.owner;
        frame.data[2*i] = slot.raw >> 8; frame.data[2*i+1] = slot.raw;
      }
      if (!used) { sent_[group] = true; continue; }
      const auto next = transport_.SendStandard(frame);
      if (result == DEVICE_OK) result = next;
      sent_[group] = next == DEVICE_OK;
      for (size_t i = 0; i < 4; ++i) {
        auto& slot = slots_[group * 4 + i];
        if (!slot.state) continue;
        slot.state->last_commit_ok = next == DEVICE_OK;
        slot.state->command_pending = next != DEVICE_OK;
        if (next == DEVICE_OK && slot.relax)
          slot.state->protocol_state = MotorProtocolState::Relaxed;
      }
    }
    if (result == DEVICE_OK) open_ = false;
    return result;
  }
  int8_t EmergencyStop() { BeginCycle(); return Flush(); }
 private:
  static bool Valid(const RmCommandAddress& a) {
    RmCommandBank check;
    return check.Set(a, 0);
  }
  static size_t Index(const RmCommandAddress& a) { return static_cast<size_t>(a.group)*4+a.slot; }
  struct Slot {
    const void* owner = nullptr;
    MotorState* state = nullptr;
    uint16_t raw = 0;
    bool staged = false, relax = true;
  };
  CanTransport transport_;
  Slot slots_[12]{};
  bool sent_[3]{};
  bool open_ = false, sealed_ = false;
};
}  // namespace mr::motor

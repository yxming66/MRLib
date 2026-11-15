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
  ~RmCommandBus() { transport_.port()->ReleaseEndpoints(this); }
  RmCommandBus(const RmCommandBus&) = delete;
  RmCommandBus& operator=(const RmCommandBus&) = delete;
  CanPort* port() const { return transport_.port(); }
  bool CanRegister(const RmCommandAddress& address) const {
    return Valid(address) && !slots_[Index(address)].owner;
  }
  int8_t Register(const RmCommandAddress& address, const void* owner, MotorState& state) {
    if (!owner || !CanRegister(address)) return DEVICE_ERR;
    const uint32_t key = GroupId(static_cast<size_t>(address.group));
    if (!transport_.port()->ClaimEndpoints(this, &key, 1)) return DEVICE_ERR;
    auto& slot = slots_[Index(address)];
    slot.owner = owner;
    slot.state = &state;
    return DEVICE_OK;
  }
  void Release(const void* owner) {
    for (auto& slot : slots_)
      if (slot.owner == owner) slot = {};
    for (size_t group = 0; group < 3; ++group) {
      bool used = false;
      for (size_t i = 0; i < 4; ++i) used = used || slots_[group * 4 + i].owner;
      if (!used) transport_.port()->ReleaseEndpoint(this, GroupId(group));
    }
  }
  void BeginCycle() {
    open_ = true;
    sealed_ = false;
    for (auto& done : sent_) done = false;
    for (auto& slot : slots_) {
      slot.raw = 0;
      slot.staged = false;
      slot.relax = true;
      if (slot.state) slot.state->command_pending = false;
    }
  }
  int8_t Stage(const void* owner, int16_t raw, bool relax) {
    if (!open_ || sealed_) return DEVICE_ERR;
    for (auto& slot : slots_)
      if (slot.owner == owner) {
        slot.raw = static_cast<uint16_t>(raw);
        slot.staged = true;
        slot.relax = relax;
        slot.state->command_pending = true;
        slot.state->last_commit_ok = false;
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
    for (auto& slot : slots_)
      if (slot.owner == owner) {
        slot.raw = 0;
        slot.staged = false;
        slot.relax = true;
        slot.state->command_pending = false;
      }
  }
  int8_t Flush() {
    if (!open_) return DEVICE_OK;
    sealed_ = true;
    int8_t result = DEVICE_OK;
    for (size_t group = 0; group < 3; ++group) {
      if (sent_[group]) continue;
      const auto next = FlushGroup(group);
      if (result == DEVICE_OK) result = next;
    }
    if (result == DEVICE_OK) open_ = false;
    return result;
  }

  // Clears and sends one command group without disturbing other staged groups.
  // Call after BeginCycle; Flush can then send the remaining groups.
  int8_t EmergencyStop(RmCommandGroup group) {
    if (!open_) return DEVICE_ERR;
    const size_t index = static_cast<size_t>(group);
    if (index >= 3U) return DEVICE_ERR;
    for (size_t i = 0; i < 4U; ++i) {
      auto& slot = slots_[index * 4U + i];
      slot.raw = 0U;
      slot.staged = false;
      slot.relax = true;
      if (slot.state) {
        slot.state->command_pending = true;
        slot.state->last_commit_ok = false;
      }
    }
    sent_[index] = false;
    return FlushGroup(index);
  }

  int8_t EmergencyStop() {
    BeginCycle();
    return Flush();
  }

 private:
  int8_t FlushGroup(size_t group) {
    bool used = false;
    CanFrame frame{};
    frame.id = GroupId(group);
    frame.dlc = 8;
    for (size_t i = 0; i < 4U; ++i) {
      const auto& slot = slots_[group * 4U + i];
      used = used || slot.owner;
      frame.data[2U * i] = slot.raw >> 8U;
      frame.data[2U * i + 1U] = slot.raw;
    }
    if (!used) {
      sent_[group] = true;
      return DEVICE_OK;
    }
    const int8_t result = transport_.SendStandard(frame);
    sent_[group] = result == DEVICE_OK;
    for (size_t i = 0; i < 4U; ++i) {
      auto& slot = slots_[group * 4U + i];
      if (!slot.state) continue;
      slot.state->last_commit_ok = result == DEVICE_OK;
      slot.state->command_pending = result != DEVICE_OK;
      if (result == DEVICE_OK && slot.relax)
        slot.state->protocol_state = MotorProtocolState::Relaxed;
    }
    return result;
  }

  static uint32_t GroupId(size_t group) {
    return group == 0 ? 0x200 : (group == 1 ? 0x1ff : 0x2ff);
  }
  static bool Valid(const RmCommandAddress& a) {
    RmCommandBank check;
    return check.Set(a, 0);
  }
  static size_t Index(const RmCommandAddress& a) {
    return static_cast<size_t>(a.group) * 4 + a.slot;
  }
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

#pragma once

// Pure RM current-command group mapping and payload storage. The protocol
// adapter owns the bus-indexed banks and performs the actual CAN transmission.

#include <cstddef>
#include <cstdint>

#include "device/motor/protocol/protocol_types.hpp"

namespace mr::motor {

enum class RmCommandGroup : uint8_t {
  k200 = 0U,
  k1ff,
  k2ff,
  kCount,
};

struct RmCommandAddress {
  uint32_t frame_id = 0U;
  RmCommandGroup group = RmCommandGroup::k200;
  uint8_t slot = 0U;
};

inline bool ResolveRmCommandAddress(RmModule module, uint16_t feedback_id,
                                   RmCommandAddress* address) {
  if (address == nullptr) {
    return false;
  }

  switch (module) {
    case RmModule::M2006:
    case RmModule::M3508:
      if (feedback_id >= 0x201U && feedback_id <= 0x204U) {
        *address = {0x200U, RmCommandGroup::k200,
                    static_cast<uint8_t>(feedback_id - 0x201U)};
        return true;
      }
      if (feedback_id >= 0x205U && feedback_id <= 0x208U) {
        *address = {0x1ffU, RmCommandGroup::k1ff,
                    static_cast<uint8_t>(feedback_id - 0x205U)};
        return true;
      }
      return false;
    case RmModule::Gm6020:
      if (feedback_id >= 0x205U && feedback_id <= 0x208U) {
        *address = {0x1ffU, RmCommandGroup::k1ff,
                    static_cast<uint8_t>(feedback_id - 0x205U)};
        return true;
      }
      if (feedback_id >= 0x209U && feedback_id <= 0x20bU) {
        *address = {0x2ffU, RmCommandGroup::k2ff,
                    static_cast<uint8_t>(feedback_id - 0x209U)};
        return true;
      }
      return false;
  }
  return false;
}

class RmCommandBank final {
 public:
  static constexpr std::size_t kGroupCount =
      static_cast<std::size_t>(RmCommandGroup::kCount);
  static constexpr std::size_t kSlotsPerGroup = 4U;

  bool Set(const RmCommandAddress& address, uint16_t raw_current) {
    const std::size_t group = static_cast<std::size_t>(address.group);
    if (group >= kGroupCount || address.slot >= kSlotsPerGroup ||
        !IsValidFrameId(address.group, address.frame_id)) {
      return false;
    }
    values_[group][address.slot] = raw_current;
    return true;
  }

  bool Encode(const RmCommandAddress& address, uint8_t* data,
              std::size_t capacity) const {
    const std::size_t group = static_cast<std::size_t>(address.group);
    if (data == nullptr || capacity < kSlotsPerGroup * 2U ||
        group >= kGroupCount || address.slot >= kSlotsPerGroup ||
        !IsValidFrameId(address.group, address.frame_id)) {
      return false;
    }
    for (std::size_t slot = 0U; slot < kSlotsPerGroup; ++slot) {
      const uint16_t value = values_[group][slot];
      data[slot * 2U] = static_cast<uint8_t>(value >> 8U);
      data[slot * 2U + 1U] = static_cast<uint8_t>(value);
    }
    return true;
  }

 private:
  static bool IsValidFrameId(RmCommandGroup group, uint32_t frame_id) {
    switch (group) {
      case RmCommandGroup::k200:
        return frame_id == 0x200U;
      case RmCommandGroup::k1ff:
        return frame_id == 0x1ffU;
      case RmCommandGroup::k2ff:
        return frame_id == 0x2ffU;
      case RmCommandGroup::kCount:
        return false;
    }
    return false;
  }

  uint16_t values_[kGroupCount][kSlotsPerGroup]{};
};

}  // namespace mr::motor

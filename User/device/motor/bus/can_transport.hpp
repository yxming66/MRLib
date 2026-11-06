#pragma once
#include <cstddef>
#include <cstdint>

#include "device/device.h"
namespace mr::motor {
inline constexpr int8_t kCanNoFrame = 1;
struct CanFrame {
  enum class Format : uint8_t { Standard, Extended };
  uint32_t id = 0;
  Format format = Format::Standard;
  uint8_t dlc = 0;
  uint8_t data[8]{};  // Motor protocols use classic CAN, even on FDCAN hardware.
  uint32_t timestamp_ms = 0;
  uint32_t sequence = 0;
};
using CanIdParser = uint32_t (*)(uint32_t, CanFrame::Format);
// Board owns IRQs/storage and copies TX bytes before returning. Empty RX is
// kCanNoFrame; errors are negative. NowMs and RX timestamps share one clock.
class CanPort {
 public:
  CanPort() = default;
  CanPort(const CanPort&) = delete;
  CanPort& operator=(const CanPort&) = delete;
  virtual ~CanPort() = default;
  virtual uint32_t NowMs() const = 0;
  virtual int8_t RegisterLatest(uint32_t id) = 0;
  virtual int8_t RegisterQueue(uint32_t id, uint8_t size) = 0;
  virtual int8_t RegisterIdParser(CanIdParser parser) = 0;
  virtual int8_t ReadLatest(uint32_t id, CanFrame& frame) = 0;
  virtual int8_t ReadNext(uint32_t id, CanFrame& frame) = 0;
  virtual int8_t Send(const CanFrame& frame) = 0;

  // Software ownership only; register from a single assembly/control context.
  // Standard IDs use the ID directly; extended subscriptions and LZ target
  // identities use disjoint namespaces. All keys are claimed atomically.
  static constexpr uint32_t kExtendedEndpoint = 0x20000000UL;
  static constexpr uint32_t kLzTargetEndpoint = 0x40000000UL;
  bool ClaimEndpoints(const void* owner, const uint32_t* keys, size_t count) {
    if (!owner || !keys) return false;
    size_t free = 0, needed = 0;
    for (const auto& claim : claims_)
      if (!claim.owner) ++free;
    for (size_t i = 0; i < count; ++i) {
      bool found = false;
      for (const auto& claim : claims_)
        if (claim.owner && claim.key == keys[i]) {
          if (claim.owner != owner) return false;
          found = true;
        }
      for (size_t j = 0; j < i; ++j)
        if (keys[j] == keys[i]) found = true;
      if (!found) ++needed;
    }
    if (needed > free) return false;
    for (size_t i = 0; i < count; ++i) {
      bool found = false;
      for (const auto& claim : claims_)
        if (claim.owner && claim.key == keys[i]) found = true;
      if (found) continue;
      for (auto& claim : claims_)
        if (!claim.owner) {
          claim = {owner, keys[i]};
          break;
        }
    }
    return true;
  }
  void ReleaseEndpoint(const void* owner, uint32_t key) {
    for (auto& claim : claims_)
      if (claim.owner == owner && claim.key == key) claim = {};
  }
  void ReleaseEndpoints(const void* owner) {
    for (auto& claim : claims_)
      if (claim.owner == owner) claim = {};
  }

 private:
  struct Claim {
    const void* owner = nullptr;
    uint32_t key = 0;
  };
  Claim claims_[64]{};
};
class CanTransport final {
 public:
  explicit constexpr CanTransport(CanPort* port = nullptr) : port_(port) {}
  CanPort* port() const { return port_; }
  bool is_valid() const { return port_ != nullptr; }
  uint32_t NowMs() const { return port_ ? port_->NowMs() : 0; }
  int8_t RegisterLatest(uint32_t id) const {
    return port_ ? port_->RegisterLatest(id) : DEVICE_ERR_NULL;
  }
  int8_t RegisterQueue(uint32_t id, uint8_t size) const {
    return port_ ? port_->RegisterQueue(id, size) : DEVICE_ERR_NULL;
  }
  int8_t RegisterIdParser(CanIdParser parser) const {
    return port_ && parser ? port_->RegisterIdParser(parser) : DEVICE_ERR_NULL;
  }
  int8_t ReadLatest(uint32_t id, CanFrame* frame) const { return Read(id, frame, true); }
  int8_t ReadNext(uint32_t id, CanFrame* frame) const { return Read(id, frame, false); }
  int8_t SendStandard(const CanFrame& frame) const {
    return frame.format == CanFrame::Format::Standard ? Send(frame) : DEVICE_ERR;
  }
  int8_t SendExtended(const CanFrame& frame) const {
    return frame.format == CanFrame::Format::Extended ? Send(frame) : DEVICE_ERR;
  }

 private:
  static bool Valid(const CanFrame& f) {
    return f.dlc <= sizeof(f.data) &&
           ((f.format == CanFrame::Format::Standard && f.id <= 0x7ff) ||
            (f.format == CanFrame::Format::Extended && f.id <= 0x1fffffff));
  }
  int8_t Read(uint32_t id, CanFrame* frame, bool latest) const {
    if (!port_ || !frame) return DEVICE_ERR_NULL;
    CanFrame candidate{};
    const auto result = latest ? port_->ReadLatest(id, candidate) : port_->ReadNext(id, candidate);
    if (result != DEVICE_OK) return result;
    if (!Valid(candidate)) return DEVICE_ERR;
    *frame = candidate;
    return DEVICE_OK;
  }
  int8_t Send(const CanFrame& frame) const {
    return !port_ ? DEVICE_ERR_NULL : (Valid(frame) ? port_->Send(frame) : DEVICE_ERR);
  }
  CanPort* port_;
};
}  // namespace mr::motor

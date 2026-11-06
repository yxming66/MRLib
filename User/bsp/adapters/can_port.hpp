#pragma once
#include "bsp/can.h"
#include "device/motor/bus/can_transport.hpp"
namespace mr::board {
// Optional adapter for the existing BSP. The project initializes CAN and
// supplies its monotonic millisecond clock; no implicit hardware startup.
class BspCanPort final : public motor::CanPort {
 public:
  using Clock = uint32_t (*)();
  BspCanPort(BSP_CAN_t bus, Clock clock) : bus_(bus), clock_(clock) {}
  uint32_t NowMs() const override { return clock_ ? clock_() : 0; }
  int8_t RegisterLatest(uint32_t id) override {
    return Valid() ? BSP_CAN_RegisterLatestId(bus_, id) : DEVICE_ERR;
  }
  int8_t RegisterQueue(uint32_t id, uint8_t size) override {
    return Valid() ? BSP_CAN_RegisterId(bus_, id, size) : DEVICE_ERR;
  }
  int8_t RegisterIdParser(motor::CanIdParser parser) override {
    if (!Valid() || !parser) return DEVICE_ERR;
    for (auto existing : parsers_) if (existing == parser) return DEVICE_OK;
    for (auto& slot : parsers_) if (!slot) {
      const auto result = BSP_CAN_RegisterIdParser(Parse);
      if (result == BSP_OK) slot = parser;
      return result;
    }
    return DEVICE_ERR;
  }
  int8_t ReadLatest(uint32_t id, motor::CanFrame& frame) override { return Read(id, frame, true); }
  int8_t ReadNext(uint32_t id, motor::CanFrame& frame) override { return Read(id, frame, false); }
  int8_t Send(const motor::CanFrame& frame) override {
    if (!Valid() || frame.dlc > 8) return DEVICE_ERR;
    if (frame.format == motor::CanFrame::Format::Standard) {
      if (frame.id > 0x7ff) return DEVICE_ERR;
      BSP_CAN_StdDataFrame_t native{};
      native.id = frame.id; native.dlc = frame.dlc;
      for (uint8_t i = 0; i < frame.dlc; ++i) native.data[i] = frame.data[i];
      return BSP_CAN_TransmitStdDataFrame(bus_, &native);
    }
    if (frame.format != motor::CanFrame::Format::Extended || frame.id > 0x1fffffff) return DEVICE_ERR;
    BSP_CAN_ExtDataFrame_t native{};
    native.id = frame.id; native.dlc = frame.dlc;
    for (uint8_t i = 0; i < frame.dlc; ++i) native.data[i] = frame.data[i];
    return BSP_CAN_TransmitExtDataFrame(bus_, &native);
  }
 private:
  bool Valid() const { return static_cast<unsigned>(bus_) < BSP_CAN_NUM && clock_; }
  // The legacy BSP has a process-wide parser registry. These pure callbacks
  // have static lifetime; register only during single-threaded board assembly.
  static uint32_t Parse(uint32_t id, BSP_CAN_FrameType_t type) {
    if (type != BSP_CAN_FRAME_STD_DATA && type != BSP_CAN_FRAME_EXT_DATA) return id;
    for (auto parser : parsers_) if (parser) {
      const auto result = parser(id, type == BSP_CAN_FRAME_EXT_DATA
          ? motor::CanFrame::Format::Extended : motor::CanFrame::Format::Standard);
      if (result != id) return result;
    }
    return id;
  }
  int8_t Read(uint32_t id, motor::CanFrame& frame, bool latest) {
    if (!Valid()) return DEVICE_ERR;
    BSP_CAN_Message_t native{};
    const auto result = latest ? BSP_CAN_GetLatestMessage(bus_, id, &native)
        : BSP_CAN_GetMessage(bus_, id, &native, BSP_CAN_TIMEOUT_IMMEDIATE);
    // The legacy manager uses generic BSP_ERR for an empty queue/latest slot.
    if (result == BSP_ERR) return motor::kCanNoFrame;
    if (result != BSP_OK) return result;
    if (native.dlc > 8 || (native.frame_type != BSP_CAN_FRAME_STD_DATA &&
        native.frame_type != BSP_CAN_FRAME_EXT_DATA)) return DEVICE_ERR;
    frame = {};
    frame.id = native.original_id;
    frame.format = native.frame_type == BSP_CAN_FRAME_EXT_DATA
        ? motor::CanFrame::Format::Extended : motor::CanFrame::Format::Standard;
    frame.dlc = native.dlc; frame.timestamp_ms = native.timestamp; frame.sequence = native.sequence;
    for (uint8_t i = 0; i < native.dlc; ++i) frame.data[i] = native.data[i];
    return DEVICE_OK;
  }
  BSP_CAN_t bus_;
  Clock clock_;
  inline static motor::CanIdParser parsers_[8]{};
};
}  // namespace mr::board

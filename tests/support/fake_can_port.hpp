#pragma once
#include <map>
#include <deque>
#include <vector>
#include "device/motor/bus/can_transport.hpp"
struct FakeCanPort final : mr::motor::CanPort {
  using Frame = mr::motor::CanFrame;
  uint32_t now = 0;
  bool fail_send = false, fail_read = false, fail_registration = false;
  std::map<uint32_t, Frame> latest;
  std::map<uint32_t, std::deque<Frame>> queues;
  std::vector<Frame> sent;
  std::vector<mr::motor::CanIdParser> parsers;
  uint32_t NowMs() const override { return now; }
  int8_t RegisterLatest(uint32_t id) override {
    if (fail_registration) return DEVICE_ERR;
    latest.emplace(id, Frame{}); return 0;
  }
  int8_t RegisterQueue(uint32_t id, uint8_t) override { queues.emplace(id, std::deque<Frame>{}); return 0; }
  int8_t RegisterIdParser(mr::motor::CanIdParser parser) override { parsers.push_back(parser); return 0; }
  int8_t ReadLatest(uint32_t id, Frame& frame) override {
    if (fail_read) return DEVICE_ERR;
    auto it = latest.find(id);
    if (it == latest.end() || it->second.dlc == 0) return mr::motor::kCanNoFrame;
    frame = it->second; return 0;
  }
  int8_t ReadNext(uint32_t id, Frame& frame) override {
    if (fail_read) return DEVICE_ERR;
    auto it = queues.find(id);
    if (it == queues.end() || it->second.empty()) return mr::motor::kCanNoFrame;
    frame = it->second.front(); it->second.pop_front(); return 0;
  }
  int8_t Send(const Frame& frame) override {
    if (fail_send) { fail_send = false; return DEVICE_ERR; }
    sent.push_back(frame); return 0;
  }
  void Push(Frame frame) {
    uint32_t key = frame.id;
    for (auto parser : parsers) {
      const auto parsed = parser(frame.id, frame.format);
      if (parsed != frame.id) { key = parsed; break; }
    }
    latest[key] = frame;
    auto queue = queues.find(key);
    if (queue != queues.end()) queue->second.push_back(frame);
  }
};

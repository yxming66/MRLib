#include <cassert>
#include <cstdio>
#include <cstring>

#include "component/communication/mrlink/mrlink.hpp"
#include "device/mrlink/mrlink_lite.hpp"
struct Message {
  static constexpr uint8_t topic = 0x32;
  uint16_t id;
  float value;
};
struct SameSize {
  uint16_t id;
  float value;
};
struct TooLarge {
  uint8_t data[64];
};
namespace mr::link {
template <>
struct MessageCodec<Message> {
  static constexpr uint16_t size = 6;
  static bool Encode(const Message& v, uint8_t* p, uint16_t n) {
    if (n != size) return false;
    wire::PutU16(p, v.id);
    wire::PutF32(p + 2, v.value);
    return true;
  }
  static bool Decode(const uint8_t* p, uint16_t n, Message* v) {
    if (n != size || wire::GetU16(p) == 0xffff) return false;
    v->id = wire::GetU16(p);
    v->value = wire::GetF32(p + 2);
    return true;
  }
};
template <>
struct MessageCodec<SameSize> : LegacyNativeCodec<SameSize> {};
template <>
struct MessageCodec<TooLarge> : LegacyNativeCodec<TooLarge> {};
}  // namespace mr::link
static unsigned calls;
static void Receive(const Message& m, void* ctx) {
  assert(m.id == 0x1234 && m.value == 1.5f);
  ++calls;
  auto* link = static_cast<mr::link::BasicInstance<1>*>(ctx);
  if (link) {
    assert(link->Off(Message::topic) == MRLINK_ERR_BUSY);
    assert(link->OnLatest<Message>(Message::topic) == MRLINK_ERR_BUSY);
  }
}
static void IgnoreLarge(const TooLarge&, void*) {}
struct Sink {
  uint8_t data[70];
  uint16_t len;
};
static bool CopyWriter(const uint8_t* p, uint16_t n, void* ctx) {
  auto* sink = static_cast<Sink*>(ctx);
  std::memcpy(sink->data, p, n);
  sink->len = n;
  return true;
}
int main() {
  mr::link::BasicInstance<1> link;
  uint8_t rx[256], tx[70];
  const MrLink_Config_t cfg = {6, true, 0, 0};
  assert(link.Init(&cfg, rx, sizeof(rx), tx, sizeof(tx)) == MRLINK_OK);
  assert(link.OnLatest<Message>(Message::topic, Receive, &link) == MRLINK_OK);
  Sink sink{};
  link.SetWriter(CopyWriter, &sink);
  assert(link.Publish(Message{0x1234, 1.5f}));
  const uint8_t wire[] = {0x34, 0x12, 0x00, 0x00, 0xc0, 0x3f};
  assert(sink.len == 12 && std::memcmp(sink.data + 4, wire, 6) == 0);
  assert(link.PushBytes(sink.data, sink.len) == MRLINK_OK);
  assert(link.Dispatch() == MRLINK_OK && calls == 1);
  Message latest{};
  uint32_t seq = 0;
  assert(link.CopyLatest(&latest, &seq) && seq == 1 && latest.value == 1.5f);
  SameSize other{};
  assert(!link.CopyLatest(Message::topic, &other));
  // Failed replacement must preserve both the old callback and its cache.
  assert(link.On<TooLarge>(Message::topic, IgnoreLarge) == MRLINK_ERR_ARGS);
  assert(link.CopyLatest(&latest, &seq) && seq == 1);
  assert(link.Init(&cfg, rx, 2, tx, 2) == MRLINK_ERR_ARGS);
  assert(link.CopyLatest(&latest, &seq));
  assert(link.PushBytes(sink.data, sink.len) == MRLINK_OK);
  assert(link.Dispatch() == MRLINK_OK && calls == 2);
  assert(link.Off(Message::topic) == MRLINK_OK && !link.CopyLatest(&latest));
  assert(link.OnLatest<Message>() == MRLINK_OK);
  // Explicit codec rejects malformed message without updating latest cache.
  assert(link.Build(Message{0xffff, 2.0f}) != 0);
  assert(link.PushBytes(tx, 12) == MRLINK_OK && link.Dispatch() == MRLINK_OK);
  assert(!link.CopyLatest(&latest));
  assert(link.Reset() == MRLINK_OK && !link.CopyLatest(&latest));
  mr::link::lite::Instance<1> alias;
  (void)alias;
  std::puts("MRLink C++ codec/slot lifecycle tests passed");
}

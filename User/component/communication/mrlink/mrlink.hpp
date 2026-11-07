#pragma once
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include "component/communication/mrlink/mrlink.h"

namespace mr::link {
using Topic = uint8_t;
/* Must copy/consume all bytes before returning true. A retaining DMA writer is
 * invalid: use the owned UART/USB channel adapter or your own TX queue. */
using Writer = bool (*)(const uint8_t *, uint16_t, void *);
using ErrorHandler = void (*)(const MrLink_ErrorInfo_t &, void *);
using EmptyHandler = void (*)(void *);
struct Transport {
  void *ctx = nullptr;
  Writer send = nullptr;
  uint16_t (*pop_rx)(void *, uint8_t *, uint16_t) = nullptr;
  int8_t (*start_rx)(void *) = nullptr;
};

template <typename T> struct MessageTraits;
/* Every message explicitly defines size, Encode(value, bytes, size) and
 * Decode(bytes, size, &value). There is NO automatic native struct codec. */
template <typename T> struct MessageCodec;
template <typename T> struct LegacyNativeCodec {
  static_assert(std::is_trivially_copyable_v<T>, "legacy payload must be trivially copyable");
  static constexpr uint16_t size = sizeof(T);
  static bool Encode(const T &v, uint8_t *out, uint16_t n) {
    if (!out || n != size) return false;
    std::memcpy(out, &v, size); return true;
  }
  static bool Decode(const uint8_t *bytes, uint16_t n, T *out) {
    if (!bytes || !out || n != size) return false;
    std::memcpy(out, bytes, size); return true;
  }
};
namespace wire {
inline void PutU16(uint8_t *p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8u); }
inline uint16_t GetU16(const uint8_t *p) { return uint16_t(p[0] | (uint16_t(p[1]) << 8u)); }
inline void PutU32(uint8_t *p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8u*i));
}
inline uint32_t GetU32(const uint8_t *p) {
  uint32_t v = 0; for (unsigned i = 0; i < 4; ++i) v |= uint32_t(p[i]) << (8u*i); return v;
}
inline void PutF32(uint8_t *p, float v) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "wire float requires IEEE754 binary32");
  uint32_t u; std::memcpy(&u, &v, 4); PutU32(p, u);
}
inline float GetF32(const uint8_t *p) {
  uint32_t u = GetU32(p); float v; std::memcpy(&v, &u, 4); return v;
}
}  // namespace wire
namespace detail {
template <typename T, typename = void> struct TopicResolver {
  static constexpr Topic Value() { return static_cast<Topic>(T::topic); }
};
template <typename T> struct TopicResolver<T, std::void_t<decltype(MessageTraits<T>::topic)>> {
  static constexpr Topic Value() { return static_cast<Topic>(MessageTraits<T>::topic); }
};
template <typename T> inline const uint8_t TypeKey = 0;
}  // namespace detail
template <typename T> constexpr Topic TopicOf() { return detail::TopicResolver<T>::Value(); }
#define MRLINK_TOPIC(PayloadType, TopicValue) \
  namespace mr::link { template <> struct MessageTraits<PayloadType> { \
  static constexpr Topic topic = static_cast<Topic>(TopicValue); }; }
#define MRLINK_MESSAGE(PayloadType, TopicValue) MRLINK_TOPIC(PayloadType, TopicValue)

/* Fixed function/context slots. Configure and dispatch from one task. Cache
 * snapshot may use the same injected synchronization as RX. No allocation,
 * std::function, typed pointers into byte arrays, or HAL includes. */
template <uint8_t MaxHandlers = 16u> class BasicInstance {
 public:
  static_assert(MaxHandlers > 0 && MaxHandlers <= MRLINK_MAX_HANDLERS, "invalid handler capacity");
  BasicInstance() = default;
  BasicInstance(const BasicInstance &) = delete;
  BasicInstance &operator=(const BasicInstance &) = delete;
  int8_t Init(const MrLink_Config_t *cfg, uint8_t *rx, uint16_t rn, uint8_t *tx, uint16_t tn) {
    const int8_t status = MrLink_Init(&proto_, cfg, rx, rn, tx, tn);
    if (status != MRLINK_OK) return status;
    for (auto &slot : slots_) slot = Slot{};
    sync_ = {}; error_ = nullptr; error_ctx_ = nullptr;
    tx_ = tx; transport_ = {}; return status;
  }
  int8_t SetSynchronization(const MrLink_Synchronization_t *sync) {
    const int8_t status = MrLink_SetSynchronization(&proto_, sync);
    if (status == MRLINK_OK) sync_ = sync ? *sync : MrLink_Synchronization_t{};
    return status;
  }
  int8_t Reset() {
    const int8_t status = MrLink_Reset(&proto_);
    if (status == MRLINK_OK) for (auto &slot : slots_) slot = Slot{};
    return status;
  }
  int8_t PushBytes(const uint8_t *data, uint16_t n) { return MrLink_PushBytes(&proto_, data, n); }
  int8_t Dispatch() { return MrLink_Dispatch(&proto_); }
  int8_t Parse(uint8_t *cmd, const uint8_t **data, uint16_t *len) {
    return MrLink_Parse(&proto_, cmd, data, len);
  }
  uint16_t Build(Topic topic, const uint8_t *data, uint16_t n) {
    return MrLink_Build(&proto_, topic, data, n);
  }
  uint16_t BuildEmpty(Topic topic) { return Build(topic, nullptr, 0); }
  template <typename T> uint16_t Build(Topic topic, const T &value) {
    static_assert(MessageCodec<T>::size <= MRLINK_MAX_PAYLOAD_DEFAULT, "codec too large");
    uint8_t bytes[MRLINK_MAX_PAYLOAD_DEFAULT]{};
    if (!MessageCodec<T>::Encode(value, bytes, MessageCodec<T>::size)) return 0;
    return Build(topic, bytes, MessageCodec<T>::size);
  }
  template <typename T> uint16_t Build(const T &v) { return Build(TopicOf<T>(), v); }
  bool PublishEmpty(Topic topic) { return Write(BuildEmpty(topic)); }
  bool Publish(Topic topic, const uint8_t *bytes, uint16_t n) { return Write(Build(topic, bytes, n)); }
  template <typename T> bool Publish(Topic topic, const T &v) { return Write(Build(topic, v)); }
  template <typename T> bool Publish(const T &v) { return Publish(TopicOf<T>(), v); }
  const uint8_t *TxBuffer() const { return tx_; }
  void SetWriter(Writer fn, void *ctx = nullptr) { transport_ = {}; transport_.send = fn; transport_.ctx = ctx; }
  void ClearWriter() { transport_ = {}; }
  bool HasWriter() const { return transport_.send != nullptr; }
  void BindTransport(const Transport &transport) { transport_ = transport; }
  int8_t StartRx() {
    return transport_.start_rx ? transport_.start_rx(transport_.ctx) : MRLINK_ERR_ARGS;
  }
  uint16_t PollRx(uint8_t *scratch, uint16_t capacity, bool dispatch = true) {
    const uint16_t space = MrLink_RxSpace(&proto_);
    uint16_t n = 0;
    if (transport_.pop_rx && scratch && capacity && space) {
      if (capacity > space) capacity = space;
      n = transport_.pop_rx(transport_.ctx, scratch, capacity);
      if (n > capacity) return 0; // transport contract violation
      if (n) (void)PushBytes(scratch, n);
    }
    if (dispatch) (void)Dispatch(); // drain work left by the previous budget even without new RX
    return n;
  }
  int8_t OnError(ErrorHandler fn, void *ctx = nullptr) {
    const int8_t status = MrLink_SetErrorHandler(&proto_, fn ? ErrorThunk : nullptr, this);
    if (status == MRLINK_OK) { error_ = fn; error_ctx_ = ctx; }
    return status;
  }
  int8_t On(Topic topic, EmptyHandler fn, void *ctx = nullptr) {
    if (!fn) return Off(topic);
    Slot *slot = Allocate(topic);
    if (!slot) return MRLINK_ERR_ARGS;
    const int8_t status = MrLink_RegisterTypedHandler(&proto_, topic, 0, Receive, slot);
    if (status != MRLINK_OK) return status;
    *slot = {}; slot->owner = this; slot->used = true; slot->topic = topic;
    slot->empty = fn; slot->ctx = ctx; return status;
  }
  template <typename T> int8_t On(Topic topic, void (*fn)(const T &, void *), void *ctx = nullptr) {
    if (!fn) return Off(topic);
    return Register<T>(topic, fn, ctx, false);
  }
  template <typename T> int8_t On(void (*fn)(const T &, void *), void *ctx = nullptr) {
    return On<T>(TopicOf<T>(), fn, ctx);
  }
  template <typename T> int8_t OnLatest(Topic topic,
      void (*fn)(const T &, void *) = nullptr, void *ctx = nullptr) {
    return Register<T>(topic, fn, ctx, true);
  }
  template <typename T> int8_t OnLatest(void (*fn)(const T &, void *) = nullptr, void *ctx = nullptr) {
    return OnLatest<T>(TopicOf<T>(), fn, ctx);
  }
  template <typename T> bool CopyLatest(Topic topic, T *out, uint32_t *sequence = nullptr) const {
    if (!out) return false;
    uint8_t bytes[MRLINK_MAX_PAYLOAD_DEFAULT]; uint32_t seq;
    {
      Lock lock(sync_);
      const Slot *slot = Find(topic);
      if (!slot || !slot->cached || slot->type != &detail::TypeKey<T>) return false;
      std::memcpy(bytes, slot->bytes, MessageCodec<T>::size); seq = slot->sequence;
    }
    T value{};
    if (!MessageCodec<T>::Decode(bytes, MessageCodec<T>::size, &value)) return false;
    *out = value; if (sequence) *sequence = seq; return true;
  }
  template <typename T> bool CopyLatest(T *out, uint32_t *sequence = nullptr) const {
    return CopyLatest(TopicOf<T>(), out, sequence);
  }
  int8_t Off(Topic topic) {
    const int8_t status = MrLink_RegisterHandler(&proto_, topic, nullptr, nullptr);
    if (status != MRLINK_OK) return status;
    Lock lock(sync_);
    for (auto &slot : slots_) if (slot.used && slot.topic == topic) slot = Slot{};
    return status;
  }
  int8_t GetStatsSnapshot(MrLink_Stats_t *out) const { return MrLink_GetStatsSnapshot(&proto_, out); }
 private:
  class Lock {
   public:
    explicit Lock(const MrLink_Synchronization_t &s) : sync_(s), token_(s.enter ? s.enter(s.ctx) : 0) {}
    ~Lock() { if (sync_.exit) sync_.exit(sync_.ctx, token_); }
    Lock(const Lock &) = delete;
    Lock &operator=(const Lock &) = delete;
   private: MrLink_Synchronization_t sync_; uintptr_t token_;
  };
  struct Slot {
    BasicInstance *owner = nullptr;
    bool used = false, latest = false, cached = false;
    Topic topic = 0;
    const void *type = nullptr;
    void (*erased)() = nullptr;
    int8_t (*invoke)(Slot *, const uint8_t *, uint16_t) = nullptr;
    EmptyHandler empty = nullptr;
    void *ctx = nullptr;
    uint32_t sequence = 0;
    uint8_t bytes[MRLINK_MAX_PAYLOAD_DEFAULT]{};
  };
  Slot *Allocate(Topic topic) {
    Slot *free = nullptr;
    for (auto &s : slots_) { if (s.used && s.topic == topic) return &s; if (!s.used && !free) free = &s; }
    return free;
  }
  const Slot *Find(Topic topic) const {
    for (const auto &s : slots_) if (s.used && s.topic == topic) return &s;
    return nullptr;
  }
  template <typename T> int8_t Register(Topic topic, void (*fn)(const T &, void *), void *ctx, bool latest) {
    static_assert(MessageCodec<T>::size <= MRLINK_MAX_PAYLOAD_DEFAULT, "codec too large");
    Slot *slot = Allocate(topic);
    if (!slot) return MRLINK_ERR_ARGS;
    const int8_t status = MrLink_RegisterTypedHandler(&proto_, topic, MessageCodec<T>::size, Receive, slot);
    if (status != MRLINK_OK) return status; // existing slot/cache remains intact on failure
    Lock lock(sync_);
    *slot = {}; slot->owner = this; slot->used = true; slot->topic = topic;
    slot->latest = latest; slot->ctx = ctx; slot->erased = reinterpret_cast<void (*)()>(fn);
    slot->invoke = Invoke<T>; slot->type = &detail::TypeKey<T>;
    return status;
  }
  template <typename T> static int8_t Invoke(Slot *slot, const uint8_t *data, uint16_t n) {
    T value{};
    if (!MessageCodec<T>::Decode(data, n, &value)) return MRLINK_ERR_ARGS;
    if (slot->latest) {
      Lock lock(slot->owner->sync_);
      std::memcpy(slot->bytes, data, n); slot->cached = true; ++slot->sequence;
    }
    auto fn = reinterpret_cast<void (*)(const T &, void *)>(slot->erased);
    if (fn) fn(value, slot->ctx);
    return MRLINK_OK;
  }
  static int8_t Receive(uint8_t, const void *data, uint16_t n, void *ctx) {
    Slot *slot = static_cast<Slot *>(ctx);
    if (slot->empty) { slot->empty(slot->ctx); return MRLINK_OK; }
    return slot->invoke(slot, static_cast<const uint8_t *>(data), n);
  }
  static void ErrorThunk(const MrLink_ErrorInfo_t *info, void *ctx) {
    auto *self = static_cast<BasicInstance *>(ctx);
    self->error_(*info, self->error_ctx_);
  }
  bool Write(uint16_t n) { return n && transport_.send && transport_.send(tx_, n, transport_.ctx); }
  MrLink_t proto_{};
  Slot slots_[MaxHandlers]{};
  MrLink_Synchronization_t sync_{};
  Transport transport_{};
  uint8_t *tx_ = nullptr;
  ErrorHandler error_ = nullptr;
  void *error_ctx_ = nullptr;
};
using Instance = BasicInstance<>;
template <uint16_t RxSize = 256u, uint16_t TxSize = MRLINK_MAX_FRAME_SIZE,
          uint16_t PollSize = MRLINK_MAX_FRAME_SIZE, uint8_t MaxHandlers = 16u>
class StaticInstance : public BasicInstance<MaxHandlers> {
 public:
  static_assert(RxSize >= 5u && TxSize >= 4u && PollSize > 0u, "invalid buffer capacity");
  int8_t Begin(const MrLink_Config_t *cfg = nullptr) {
    return this->Init(cfg, rx_, RxSize, tx_, TxSize);
  }
  uint16_t Poll() { return this->PollRx(scratch_, PollSize, true); }
 private:
  uint8_t rx_[RxSize]{}, tx_[TxSize]{}, scratch_[PollSize]{};
};
template <uint16_t R = 256u, uint16_t T = MRLINK_MAX_FRAME_SIZE,
          uint16_t P = MRLINK_MAX_FRAME_SIZE, uint8_t H = 16u>
using Bus = StaticInstance<R, T, P, H>;
}  // namespace mr::link

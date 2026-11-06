#include <cassert>
#include <type_traits>

#include "component/cmd_center/cmd_center_cmsis.hpp"
#include "component/container/latest_slot.h"
#include "component/container/spsc_ringbuf.hpp"

extern "C" {
uint32_t mr_test_primask = 0;
}
int main() {
  using Ring = mr::comp::StaticSpscRingBuf<13>;
  static_assert(!std::is_copy_constructible_v<Ring> && !std::is_move_constructible_v<Ring>);
  static_assert(!std::is_copy_constructible_v<mr::comp::SpscRingBufView>);
  Ring ring;
  for (uint32_t cycle = 0; cycle < 10000; ++cycle) {
    assert(ring.TryPushObject(cycle));
    assert(ring.TryPushObject(cycle));
    assert(ring.TryPushObject(cycle));
    assert(!ring.TryPushObject(cycle));
    uint32_t out = UINT32_MAX;
    for (int i = 0; i < 3; ++i) {
      assert(ring.TryPopObject(&out));
      assert(out == cycle);
    }
    assert(!ring.TryPopObject(&out));
    assert(out == cycle);
  }
  uint32_t storage = 0, last = 0, out = 99, value = 7;
  LatestSlot_t slot{};
  assert(LatestSlot_Init(&slot, &storage, sizeof(storage)) == LATEST_SLOT_OK);
  assert(!LatestSlot_ReadLatest(&slot, &out, sizeof(out)) && out == 99);
  assert(LatestSlot_Write(&slot, &value, sizeof(value)) == LATEST_SLOT_OK);
  assert(mr_test_primask == 0);
  assert(LatestSlot_ReadIfUpdated(&slot, &out, sizeof(out), &last) && out == 7);
  assert(!LatestSlot_ReadIfUpdated(&slot, &out, sizeof(out), &last));
  LatestSlot_Reset(&slot);
  value = 8;
  assert(LatestSlot_Write(&slot, &value, sizeof(value)) == LATEST_SLOT_OK);
  assert(LatestSlot_ReadIfUpdated(&slot, &out, sizeof(out), &last) && out == 8);
  slot.seq = UINT32_MAX;
  mr_test_primask = 1;
  assert(LatestSlot_Write(&slot, &value, sizeof(value)) == LATEST_SLOT_OK);
  assert(slot.seq == 1 && mr_test_primask == 1);
  ring.Reset();
  assert(mr_test_primask == 1);
  MrTestQueue q{sizeof(uint32_t), 0, 0, 0, 0};
  cmd::CmsisQueueSink sink(&q, true);
  assert(sink(value) && q.resets == 1 && q.puts == 1);
  q.size = 1;
  assert(!sink(value) && q.resets == 1 && q.puts == 1);
  q.size = sizeof(value);
  q.reset_status = -1;
  assert(!sink(value) && q.puts == 1);
  q.reset_status = 0;
  q.put_status = -1;
  assert(!sink(value));
  q.put_status = 0;
  cmd::CmsisQueueSink regular(&q);
  const int resets = q.resets;
  assert(regular(value) && q.resets == resets);
}

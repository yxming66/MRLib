#pragma once
#include <cstddef>
#include <cstdint>

#include "bsp/dma.h"
#include "main.h"
namespace board {
/* Board supplies actual linker/MPU memory regions reachable by the peripheral's
 * DMA controller. Never include DTCM merely because the CPU can address it.
 * Cacheable regions additionally require exclusive 32-byte lines per buffer;
 * ownership of rounded padding remains with the DMA buffer until completion. */
struct DmaRegion {
  uintptr_t begin, end;
  bool cacheable;
};
struct DmaPolicy {
  const DmaRegion* regions;
  size_t count;
};
inline bool SynchronizeDma(BSP_DMA_Operation_t op, void* data, uint16_t n, void* ctx) {
  if (!data || !n || !ctx) return false;
  auto& policy = *static_cast<DmaPolicy*>(ctx);
  if (!policy.regions || op > BSP_DMA_COMPLETE_RX) return false;
  const uintptr_t start = reinterpret_cast<uintptr_t>(data);
  if (start > UINTPTR_MAX - n - 31u) return false;
  const uintptr_t end = start + n;
  for (size_t i = 0; i < policy.count; ++i) {
    const auto& r = policy.regions[i];
    if (start < r.begin || end > r.end) continue;
    if (r.cacheable) {
      const uintptr_t rounded_end = (end + 31u) & ~uintptr_t(31u);
      if (start % 32u || rounded_end > r.end) return false;
      const int32_t size = static_cast<int32_t>(rounded_end - start);
      __DSB();
      if (op == BSP_DMA_PREPARE_TX)
        SCB_CleanDCache_by_Addr(static_cast<uint32_t*>(data), size);
      else if (op == BSP_DMA_PREPARE_RX)
        SCB_CleanInvalidateDCache_by_Addr(static_cast<uint32_t*>(data), size);
      else if (op == BSP_DMA_COMPLETE_RX)
        SCB_InvalidateDCache_by_Addr(static_cast<uint32_t*>(data), size);
      else
        return false;
    }
    __DSB();
    return true;
  }
  return false;
}
}  // namespace board

#include "bsp/dma.h"

#include <stddef.h>
static BSP_DMA_Sync_t s_sync;
static void* s_ctx;
bool BSP_DMA_Configure(BSP_DMA_Sync_t sync, void* ctx) {
  if (!sync) return false;
  s_sync = sync;
  s_ctx = ctx;
  return true;
}
bool BSP_DMA_Synchronize(BSP_DMA_Operation_t op, void* data, uint16_t size) {
  return s_sync && data && size && s_sync(op, data, size, s_ctx);
}

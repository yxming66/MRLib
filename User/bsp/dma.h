#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { BSP_DMA_PREPARE_TX, BSP_DMA_PREPARE_RX, BSP_DMA_COMPLETE_RX } BSP_DMA_Operation_t;
/* Board installs once before any DMA starts. Must validate DMA-accessible memory
 * and cache-line ownership, clean TX, clean/invalidate RX before start, and
 * invalidate RX after completion as appropriate to its MPU/cache policy.
 * Runs in task OR ISR with IRQs possibly masked: bounded, no blocking/RTOS.
 * Return false on invalid memory/range; failed RX completion is not published.
 * Uncached boards still explicitly install a validating hook. No hook => reject
 * DMA, never silently access unknown H7 memory/cache. Do not change while active. */
typedef bool (*BSP_DMA_Sync_t)(BSP_DMA_Operation_t operation, void* data, uint16_t size, void* ctx);
bool BSP_DMA_Configure(BSP_DMA_Sync_t sync, void* ctx);
bool BSP_DMA_Synchronize(BSP_DMA_Operation_t operation, void* data, uint16_t size);
#ifdef __cplusplus
}
#endif

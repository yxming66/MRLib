#include "board/fdcan_endpoints.h"
#include "bsp/fdcan.h"
#include "fdcan.h"

/* Include unused FDCAN2 in layout checks: Cube initializes its shared RAM too. */
static int ValidateRam(void) {
  FDCAN_HandleTypeDef* all[] = {&hfdcan1, &hfdcan2, &hfdcan3};
  for (unsigned i = 0; i < 3; ++i) {
    const uint32_t start = all[i]->msgRam.StandardFilterSA;
    const uint32_t end = all[i]->msgRam.EndAddress;
    if (all[i]->State != HAL_FDCAN_STATE_READY || start < SRAMCAN_BASE ||
        start >= end || end > SRAMCAN_BASE + 0x2800u - 4u ||
        (start & 3u) || (end & 3u)) return 0;
    for (unsigned j = 0; j < i; ++j)
      if (start < all[j]->msgRam.EndAddress &&
          all[j]->msgRam.StandardFilterSA < end) return 0;
  }
  return 1;
}
static int ConfigureFilters(FDCAN_HandleTypeDef* bus, uint32_t fifo, int extended) {
  if (bus->Init.StdFiltersNbr != 1u || bus->Init.ExtFiltersNbr != (extended ? 1u : 0u)) return 0;
  FDCAN_FilterTypeDef filter = {0};
  filter.FilterIndex = 0;
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = fifo == FDCAN_RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
  /* Mask zero accepts data IDs; subscriptions select actual consumers. */
  if (HAL_FDCAN_ConfigFilter(bus, &filter) != HAL_OK) return 0;
  if (extended) {
    filter.IdType = FDCAN_EXTENDED_ID;
    if (HAL_FDCAN_ConfigFilter(bus, &filter) != HAL_OK) return 0;
  }
  return HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK;
}
int Board_RegisterFdcanEndpoints(void) {
  if (!ValidateRam()) return 0;
  /* The .ioc allocates no extended filter for FDCAN1, one for FDCAN3 (LZ). */
  if (hfdcan1.Init.StdFiltersNbr != 1u || hfdcan1.Init.ExtFiltersNbr != 0u ||
      hfdcan3.Init.StdFiltersNbr != 1u || hfdcan3.Init.ExtFiltersNbr != 1u) return 0;
  const BSP_FDCAN_Endpoint_t endpoints[] = {
      {"fdcan1", &hfdcan1, BSP_FDCAN_RX_FIFO0},
      {"fdcan3", &hfdcan3, BSP_FDCAN_RX_FIFO1},
  };
  if (BSP_FDCAN_Configure(endpoints, 2) != BSP_OK) return 0;
  return ConfigureFilters(&hfdcan1, FDCAN_RX_FIFO0, 0) &&
         ConfigureFilters(&hfdcan3, FDCAN_RX_FIFO1, 1);
}

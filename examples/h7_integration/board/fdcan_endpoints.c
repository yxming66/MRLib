#include "bsp/fdcan.h"
#include "fdcan.h"

static int ConfigureFilters(FDCAN_HandleTypeDef* handle, uint32_t fifo) {
  // One standard and one extended filter: their index spaces are separate.
  if (handle->Init.StdFiltersNbr < 1u || handle->Init.ExtFiltersNbr < 1u ||
      handle->msgRam.StandardFilterSA < SRAMCAN_BASE ||
      handle->msgRam.EndAddress > SRAMCAN_BASE + 0x2800u - 4u) return 0;
  FDCAN_FilterTypeDef filter = {0};
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = fifo == FDCAN_RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
  filter.IdType = FDCAN_STANDARD_ID;
  if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK) return 0;
  filter.IdType = FDCAN_EXTENDED_ID;
  if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK) return 0;
  // Broad acceptance for this integration example, software IDs select consumers.
  const uint32_t accept = fifo == FDCAN_RX_FIFO0 ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_ACCEPT_IN_RX_FIFO1;
  return HAL_FDCAN_ConfigGlobalFilter(handle, accept, accept, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) == HAL_OK;
}
int Board_RegisterFdcanEndpoints(void) {
  const BSP_FDCAN_Endpoint_t buses[] = {
      {"fdcan1", &hfdcan1, BSP_FDCAN_RX_FIFO0},
      {"fdcan2", &hfdcan2, BSP_FDCAN_RX_FIFO1},
      {"fdcan3", &hfdcan3, BSP_FDCAN_RX_FIFO1},
  };
  // Register first so overlap/alias checks run before writing any filter RAM.
  if (BSP_FDCAN_Configure(buses, 3) != BSP_OK) return 0;
  return ConfigureFilters(&hfdcan1, FDCAN_RX_FIFO0) &&
         ConfigureFilters(&hfdcan2, FDCAN_RX_FIFO1) && ConfigureFilters(&hfdcan3, FDCAN_RX_FIFO1);
}

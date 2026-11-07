#include "bsp/spi.h"

#include <stddef.h>
#include <string.h>

#include "bsp/dma.h"

static void (*SPI_Callback[BSP_SPI_NUM][BSP_SPI_CB_NUM])(void);
static BSP_SPI_Endpoint_t endpoints[BSP_SPI_NUM];
static uint8_t endpoint_count;
static bool configured;
static uint8_t* SPI_DmaRx[BSP_SPI_NUM];
static uint16_t SPI_DmaRxSize[BSP_SPI_NUM];
static bool SPI_CompleteDma(BSP_SPI_t spi, bool final) {
  uint8_t* data = SPI_DmaRx[spi];
  if (!data) return true;
  uint16_t size = SPI_DmaRxSize[spi];
  if (final) {
    SPI_DmaRx[spi] = NULL;
    SPI_DmaRxSize[spi] = 0;
  } else
    size /= 2u;
  if (BSP_DMA_Synchronize(BSP_DMA_COMPLETE_RX, data, size)) return true;
  if (SPI_Callback[spi][BSP_SPI_ERROR_CB]) SPI_Callback[spi][BSP_SPI_ERROR_CB]();
  return false;
}

static BSP_SPI_t SPI_Get(SPI_HandleTypeDef* hspi) {
  if (hspi)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (endpoints[i].handle == hspi) return (BSP_SPI_t)i;
  return BSP_SPI_ERR;
}

static void SPI_Dispatch(SPI_HandleTypeDef* hspi, BSP_SPI_Callback_t type) {
  const BSP_SPI_t spi = SPI_Get(hspi);
  if (spi == BSP_SPI_ERR) return;
  if (type == BSP_SPI_RX_CPLT_CB || type == BSP_SPI_TX_RX_CPLT_CB) {
    if (!SPI_CompleteDma(spi, true)) return;
  } else if (type == BSP_SPI_RX_HALF_CPLT_CB || type == BSP_SPI_TX_RX_HALF_CPLT_CB) {
    if (!SPI_CompleteDma(spi, false)) return;
  }
  if (SPI_Callback[spi][type]) SPI_Callback[spi][type]();
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_TX_CPLT_CB); }
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_RX_CPLT_CB); }
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_TX_RX_CPLT_CB); }
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_TX_HALF_CPLT_CB); }
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_RX_HALF_CPLT_CB); }
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_TX_RX_HALF_CPLT_CB); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_ERROR_CB); }
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi) { SPI_Dispatch(hspi, BSP_SPI_ABORT_CPLT_CB); }

SPI_HandleTypeDef* BSP_SPI_GetHandle(BSP_SPI_t spi) {
  return (unsigned)spi < endpoint_count ? endpoints[spi].handle : NULL;
}
int8_t BSP_SPI_Configure(const BSP_SPI_Endpoint_t* table, uint8_t count) {
  if (configured || !table || !count || count > BSP_SPI_NUM) return BSP_ERR;
  for (uint8_t i = 0; i < count; ++i) {
    const BSP_SPI_Endpoint_t* p = &table[i];
    if (!p->name || !p->name[0] || !p->handle || !p->handle->Instance ||
        p->handle->State != HAL_SPI_STATE_READY ||
        p->handle->Init.DataSize != SPI_DATASIZE_8BIT ||
        p->handle->Init.Mode != SPI_MODE_MASTER ||
        p->handle->Init.Direction != SPI_DIRECTION_2LINES ||
        p->handle->Init.CRCCalculation != SPI_CRCCALCULATION_DISABLE) return BSP_ERR;
    for (uint8_t j = 0; j < i; ++j)
      if (strcmp(p->name, table[j].name) == 0 ||
          p->handle->Instance == table[j].handle->Instance) return BSP_ERR;
  }
  memcpy(endpoints, table, count * sizeof(*table));
  endpoint_count = count;
  configured = true;
  return BSP_OK;
}
BSP_SPI_t BSP_SPI_FindEndpoint(const char* name) {
  if (name)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (strcmp(name, endpoints[i].name) == 0) return (BSP_SPI_t)i;
  return BSP_SPI_ERR;
}

int8_t BSP_SPI_RegisterCallback(BSP_SPI_t spi, BSP_SPI_Callback_t type, void (*callback)(void)) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || (unsigned)type >= BSP_SPI_CB_NUM || !callback) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (h->State == HAL_SPI_STATE_READY && !SPI_DmaRx[spi] && !SPI_Callback[spi][type]) {
    SPI_Callback[spi][type] = callback;
    result = BSP_OK;
  }
  if (!token) __enable_irq();
  return result;
}
int8_t BSP_SPI_UnregisterCallback(BSP_SPI_t spi, BSP_SPI_Callback_t type, void (*callback)(void)) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || (unsigned)type >= BSP_SPI_CB_NUM || !callback) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (h->State == HAL_SPI_STATE_READY && !SPI_DmaRx[spi] && SPI_Callback[spi][type] == callback) {
    SPI_Callback[spi][type] = NULL;
    result = BSP_OK;
  }
  if (!token) __enable_irq();
  return result;
}

static int8_t SPI_DmaTransfer(BSP_SPI_t spi, uint8_t* tx, uint8_t* rx, uint16_t size) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !size || (!tx && !rx) || (tx && tx == rx)) return BSP_ERR;
  if (tx && rx) {
    const uintptr_t a = (uintptr_t)tx, b = (uintptr_t)rx;
    if ((a <= b ? b - a : a - b) < size) return BSP_ERR;
  }
  // Byte buffers must not be rounded up by HAL DMA packing; neither direction
  // supports circular ownership/completion semantics here.
  if (rx && (!h->hdmarx || h->hdmarx->Init.Mode != DMA_NORMAL ||
             h->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
             h->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE)) return BSP_ERR;
  if (tx && (!h->hdmatx || h->hdmatx->Init.Mode != DMA_NORMAL ||
             h->hdmatx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
             h->hdmatx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE)) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (h->State == HAL_SPI_STATE_READY && !SPI_DmaRx[spi] &&
      (!tx || BSP_DMA_Synchronize(BSP_DMA_PREPARE_TX, tx, size)) &&
      (!rx || BSP_DMA_Synchronize(BSP_DMA_PREPARE_RX, rx, size))) {
    HAL_StatusTypeDef status;
    if (tx && rx)
      status = HAL_SPI_TransmitReceive_DMA(h, tx, rx, size);
    else if (tx)
      status = HAL_SPI_Transmit_DMA(h, tx, size);
    else
      status = HAL_SPI_Receive_DMA(h, rx, size);
    if (status == HAL_OK) {
      SPI_DmaRx[spi] = rx;
      SPI_DmaRxSize[spi] = rx ? size : 0;
      result = BSP_OK;
    }
  }
  if (!token) __enable_irq();
  return result;
}
int8_t BSP_SPI_Transmit(BSP_SPI_t spi, uint8_t* data, uint16_t size, bool dma) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !data || !size) return BSP_ERR;
  if (dma) return SPI_DmaTransfer(spi, data, NULL, size);
  if (SPI_DmaRx[spi]) return BSP_ERR;
  return HAL_SPI_Transmit(h, data, size, 20u) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_SPI_Receive(BSP_SPI_t spi, uint8_t* data, uint16_t size, bool dma) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !data || !size) return BSP_ERR;
  if (dma) return SPI_DmaTransfer(spi, NULL, data, size);
  if (SPI_DmaRx[spi]) return BSP_ERR;
  return HAL_SPI_Receive(h, data, size, 20u) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_SPI_TransmitReceive(BSP_SPI_t spi, uint8_t* tx, uint8_t* rx, uint16_t size, bool dma) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !tx || !rx || !size) return BSP_ERR;
  if (dma) return SPI_DmaTransfer(spi, tx, rx, size);
  if (SPI_DmaRx[spi]) return BSP_ERR;
  return HAL_SPI_TransmitReceive(h, tx, rx, size, 20u) == HAL_OK ? BSP_OK : BSP_ERR;
}

uint8_t BSP_SPI_MemReadByte(BSP_SPI_t spi, uint8_t reg) {
  if ((unsigned)spi >= BSP_SPI_NUM) return 0xFF;
  uint8_t tmp[2] = {reg | 0x80, 0x00};
  /* The buffer is local, so a DMA transaction would outlive this function. */
  if (BSP_SPI_TransmitReceive(spi, tmp, tmp, 2u, false) != BSP_OK) {
    return 0xFFu;
  }
  return tmp[1];
}

int8_t BSP_SPI_MemWriteByte(BSP_SPI_t spi, uint8_t reg, uint8_t data) {
  if ((unsigned)spi >= BSP_SPI_NUM || SPI_DmaRx[spi]) return BSP_ERR;
  uint8_t tmp[2] = {reg & 0x7f, data};
  /* The buffer is local, so a DMA transaction would outlive this function. */
  return BSP_SPI_Transmit(spi, tmp, 2u, false);
}

int8_t BSP_SPI_MemRead(BSP_SPI_t spi, uint8_t reg, uint8_t* data, uint16_t size) {
  if ((unsigned)spi >= BSP_SPI_NUM || SPI_DmaRx[spi]) return BSP_ERR;
  if (data == NULL || size == 0) return BSP_ERR_NULL;
  /* The register byte is a stack value; a DMA call would outlive it and the
   * following receive would race the SPI peripheral.  Memory helpers are
   * therefore deliberately synchronous.  Callers that need a DMA burst must
   * keep CS asserted and use the lower-level async API with persistent buffers. */
  reg = (uint8_t)(reg | 0x80u);
  SPI_HandleTypeDef* hspi = BSP_SPI_GetHandle(spi);
  if (hspi == NULL) return BSP_ERR_NULL;
  if (HAL_SPI_Transmit(hspi, &reg, 1u, 20u) != HAL_OK) return BSP_ERR;
  return (HAL_SPI_Receive(hspi, data, size, 20u) == HAL_OK) ? BSP_OK : BSP_ERR;
}

int8_t BSP_SPI_MemWrite(BSP_SPI_t spi, uint8_t reg, uint8_t* data, uint16_t size) {
  if ((unsigned)spi >= BSP_SPI_NUM || SPI_DmaRx[spi]) return BSP_ERR;
  if (data == NULL || size == 0) return BSP_ERR_NULL;
  reg = (uint8_t)(reg & 0x7fu);
  SPI_HandleTypeDef* hspi = BSP_SPI_GetHandle(spi);
  if (hspi == NULL) return BSP_ERR_NULL;
  if (HAL_SPI_Transmit(hspi, &reg, 1u, 20u) != HAL_OK) return BSP_ERR;
  return (HAL_SPI_Transmit(hspi, data, size, 20u) == HAL_OK) ? BSP_OK : BSP_ERR;
}

int8_t BSP_SPI_Abort(BSP_SPI_t spi) {
  SPI_HandleTypeDef* hspi = BSP_SPI_GetHandle(spi);
  if (!hspi) return BSP_ERR;
  if (HAL_SPI_Abort(hspi) != HAL_OK) return BSP_ERR;
  SPI_DmaRx[spi] = NULL;
  SPI_DmaRxSize[spi] = 0;
  return BSP_OK;
}

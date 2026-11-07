/* Includes ----------------------------------------------------------------- */
#include "bsp/spi.h"

#include <spi.h>

#include "bsp/dma.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Private define ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Private variables -------------------------------------------------------- */
static void (*SPI_Callback[BSP_SPI_NUM][BSP_SPI_CB_NUM])(void);
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

/* Private function  -------------------------------------------------------- */
static BSP_SPI_t SPI_Get(SPI_HandleTypeDef* hspi) {
  if (hspi == NULL) return BSP_SPI_ERR;
  if (hspi->Instance == SPI2)
    return BSP_SPI_BMI088;
  else
    return BSP_SPI_ERR;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[bsp_spi][BSP_SPI_TX_CPLT_CB]) {
      SPI_Callback[bsp_spi][BSP_SPI_TX_CPLT_CB]();
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR && !SPI_CompleteDma(bsp_spi, true)) return;
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_RX_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_RX_CPLT_CB]();
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR && !SPI_CompleteDma(bsp_spi, true)) return;
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_RX_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_RX_CPLT_CB]();
  }
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_HALF_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_HALF_CPLT_CB]();
  }
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR && !SPI_CompleteDma(bsp_spi, false)) return;
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_RX_HALF_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_RX_HALF_CPLT_CB]();
  }
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR && !SPI_CompleteDma(bsp_spi, false)) return;
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_RX_HALF_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_TX_RX_HALF_CPLT_CB]();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_ERROR_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_ERROR_CB]();
  }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi) {
  BSP_SPI_t bsp_spi = SPI_Get(hspi);
  if (bsp_spi != BSP_SPI_ERR) {
    if (SPI_Callback[SPI_Get(hspi)][BSP_SPI_ABORT_CPLT_CB])
      SPI_Callback[SPI_Get(hspi)][BSP_SPI_ABORT_CPLT_CB]();
  }
}

/* Exported functions ------------------------------------------------------- */
SPI_HandleTypeDef* BSP_SPI_GetHandle(BSP_SPI_t spi) {
  switch (spi) {
    case BSP_SPI_BMI088:
      return &hspi2;
    default:
      return NULL;
  }
}

int8_t BSP_SPI_RegisterCallback(BSP_SPI_t spi, BSP_SPI_Callback_t type, void (*callback)(void)) {
  if ((unsigned)spi >= BSP_SPI_NUM || (unsigned)type >= BSP_SPI_CB_NUM) return BSP_ERR;
  SPI_Callback[spi][type] = callback;
  return BSP_OK;
}

static int8_t SPI_DmaTransfer(BSP_SPI_t spi, uint8_t* tx, uint8_t* rx, uint16_t size) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !size || (!tx && !rx) || (tx && tx == rx)) return BSP_ERR;
  if (rx && (!h->hdmarx || h->hdmarx->Init.Mode != DMA_NORMAL)) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (h->State == HAL_SPI_STATE_READY &&
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
  return HAL_SPI_Transmit(h, data, size, 20u) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_SPI_Receive(BSP_SPI_t spi, uint8_t* data, uint16_t size, bool dma) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !data || !size) return BSP_ERR;
  if (dma) return SPI_DmaTransfer(spi, NULL, data, size);
  return HAL_SPI_Receive(h, data, size, 20u) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_SPI_TransmitReceive(BSP_SPI_t spi, uint8_t* tx, uint8_t* rx, uint16_t size, bool dma) {
  SPI_HandleTypeDef* h = BSP_SPI_GetHandle(spi);
  if (!h || !tx || !rx || !size) return BSP_ERR;
  if (dma) return SPI_DmaTransfer(spi, tx, rx, size);
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
  if ((unsigned)spi >= BSP_SPI_NUM) return BSP_ERR;
  uint8_t tmp[2] = {reg & 0x7f, data};
  /* The buffer is local, so a DMA transaction would outlive this function. */
  return BSP_SPI_Transmit(spi, tmp, 2u, false);
}

int8_t BSP_SPI_MemRead(BSP_SPI_t spi, uint8_t reg, uint8_t* data, uint16_t size) {
  if ((unsigned)spi >= BSP_SPI_NUM) return BSP_ERR;
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
  if ((unsigned)spi >= BSP_SPI_NUM) return BSP_ERR;
  if (data == NULL || size == 0) return BSP_ERR_NULL;
  reg = (uint8_t)(reg & 0x7fu);
  SPI_HandleTypeDef* hspi = BSP_SPI_GetHandle(spi);
  if (hspi == NULL) return BSP_ERR_NULL;
  if (HAL_SPI_Transmit(hspi, &reg, 1u, 20u) != HAL_OK) return BSP_ERR;
  return (HAL_SPI_Transmit(hspi, data, size, 20u) == HAL_OK) ? BSP_OK : BSP_ERR;
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

int8_t BSP_SPI_Abort(BSP_SPI_t spi) {
  SPI_HandleTypeDef* hspi = BSP_SPI_GetHandle(spi);
  if (!hspi) return BSP_ERR;
  if (HAL_SPI_Abort(hspi) != HAL_OK) return BSP_ERR;
  SPI_DmaRx[spi] = NULL;
  SPI_DmaRxSize[spi] = 0;
  return BSP_OK;
}

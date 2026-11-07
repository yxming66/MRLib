#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#include "bsp/bsp.h"

/* Neutral hardware bus slots. New devices do not extend this enum. */
typedef enum {
  BSP_SPI_0, BSP_SPI_1, BSP_SPI_2, BSP_SPI_3, BSP_SPI_4, BSP_SPI_5,
  BSP_SPI_NUM,
  BSP_SPI_ERR,
} BSP_SPI_t;

typedef struct {
  const char* name;
  SPI_HandleTypeDef* handle;
} BSP_SPI_Endpoint_t;

/* Startup only, after Cube SPI init. Copies the table; names/handles stay valid.
 * Requires 8-bit master, 2-line transfers, CRC disabled. Board owns pins, clock,
 * DMA and IRQ priorities; DMA requires normal mode and byte memory/peripheral alignment.
 * One owner per bus, including CS and both phases of register helpers; no lock or
 * device arbitration is provided. Unknown handle aliases never receive callbacks. */
int8_t BSP_SPI_Configure(const BSP_SPI_Endpoint_t* endpoints, uint8_t count);
BSP_SPI_t BSP_SPI_FindEndpoint(const char* name);

/* SPI支持的中断回调函数类型，具体参考HAL中定义 */
typedef enum {
  BSP_SPI_TX_CPLT_CB,
  BSP_SPI_RX_CPLT_CB,
  BSP_SPI_TX_RX_CPLT_CB,
  BSP_SPI_TX_HALF_CPLT_CB,
  BSP_SPI_RX_HALF_CPLT_CB,
  BSP_SPI_TX_RX_HALF_CPLT_CB,
  BSP_SPI_ERROR_CB,
  BSP_SPI_ABORT_CPLT_CB,
  BSP_SPI_CB_NUM,
} BSP_SPI_Callback_t;

SPI_HandleTypeDef* BSP_SPI_GetHandle(BSP_SPI_t spi);
int8_t BSP_SPI_RegisterCallback(BSP_SPI_t spi, BSP_SPI_Callback_t type, void (*callback)(void));
/* Callback changes require an idle bus; registration never replaces an existing
 * callback. Unregister requires the matching function, not NULL. */
int8_t BSP_SPI_UnregisterCallback(BSP_SPI_t spi, BSP_SPI_Callback_t type, void (*callback)(void));

int8_t BSP_SPI_Transmit(BSP_SPI_t spi, uint8_t* data, uint16_t size, bool dma);
int8_t BSP_SPI_Receive(BSP_SPI_t spi, uint8_t* data, uint16_t size, bool dma);
int8_t BSP_SPI_TransmitReceive(BSP_SPI_t spi, uint8_t* txData, uint8_t* rxData, uint16_t size,
                               bool dma);

int8_t BSP_SPI_Abort(BSP_SPI_t spi);

/* Register helpers are blocking; caller owns CS and bus across both phases. */
uint8_t BSP_SPI_MemReadByte(BSP_SPI_t spi, uint8_t reg);
int8_t BSP_SPI_MemWriteByte(BSP_SPI_t spi, uint8_t reg, uint8_t data);
int8_t BSP_SPI_MemRead(BSP_SPI_t spi, uint8_t reg, uint8_t* data, uint16_t size);
int8_t BSP_SPI_MemWrite(BSP_SPI_t spi, uint8_t reg, uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

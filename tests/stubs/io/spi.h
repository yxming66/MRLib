#pragma once
#include "usart.h"
typedef struct {
  void* Instance;
  DMA_HandleTypeDef* hdmarx;
  unsigned State;
} SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi2;
#define SPI2 ((void*)2)
#define HAL_SPI_STATE_READY 0u
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t,
                                          uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef*);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef*);

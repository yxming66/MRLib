#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "bsp/bsp.h"
#define BSP_FDCAN_DEFAULT_QUEUE_SIZE 10u
#define BSP_FDCAN_TIMEOUT_IMMEDIATE 0u
typedef enum { BSP_FDCAN_1, BSP_FDCAN_NUM, BSP_FDCAN_ERR } BSP_FDCAN_t;
typedef enum {
  BSP_FDCAN_FORMAT_STD_DATA,
  BSP_FDCAN_FORMAT_EXT_DATA,
  BSP_FDCAN_FORMAT_STD_REMOTE,
  BSP_FDCAN_FORMAT_EXT_REMOTE
} BSP_FDCAN_Format_t;
typedef enum {
  BSP_FDCAN_FRAME_STD_DATA,
  BSP_FDCAN_FRAME_EXT_DATA,
  BSP_FDCAN_FRAME_STD_REMOTE,
  BSP_FDCAN_FRAME_EXT_REMOTE
} BSP_FDCAN_FrameType_t;
typedef struct {
  BSP_FDCAN_FrameType_t frame_type;
  uint32_t original_id;
  uint8_t dlc;
  uint8_t data[64];
} BSP_FDCAN_Message_t;
int8_t BSP_FDCAN_RegisterId(BSP_FDCAN_t, uint32_t, uint8_t);
int8_t BSP_FDCAN_UnregisterId(BSP_FDCAN_t, uint32_t);
int8_t BSP_FDCAN_GetMessage(BSP_FDCAN_t, uint32_t, BSP_FDCAN_Message_t*, uint32_t);
int8_t BSP_FDCAN_Transmit(BSP_FDCAN_t, BSP_FDCAN_Format_t, uint32_t, const uint8_t*, uint8_t);

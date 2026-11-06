#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { BSP_CAN_1 = 0, BSP_CAN_2 = 1 } BSP_CAN_t;
enum { BSP_CAN_MAX_DLC = 8, BSP_CAN_TIMEOUT_IMMEDIATE = 0 };
#define BSP_CAN_NUM 2

typedef enum {
  BSP_CAN_FRAME_STD_DATA = 0,
  BSP_CAN_FRAME_EXT_DATA,
  BSP_CAN_FRAME_STD_REMOTE,
  BSP_CAN_FRAME_EXT_REMOTE,
} BSP_CAN_FrameType_t;

typedef struct {
  BSP_CAN_FrameType_t frame_type;
  uint32_t original_id;
  uint32_t parsed_id;
  uint8_t dlc;
  uint8_t data[BSP_CAN_MAX_DLC];
  uint32_t timestamp;
  uint32_t sequence;
} BSP_CAN_Message_t;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[BSP_CAN_MAX_DLC];
} BSP_CAN_StdDataFrame_t;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[BSP_CAN_MAX_DLC];
} BSP_CAN_ExtDataFrame_t;

typedef uint32_t (*BSP_CAN_IdParser_t)(uint32_t, BSP_CAN_FrameType_t);

enum { BSP_OK = 0, BSP_ERR = -1, BSP_ERR_NO_DEV = -2, BSP_ERR_NULL = -3 };

int8_t BSP_CAN_RegisterLatestId(BSP_CAN_t bus, uint32_t id);
int8_t BSP_CAN_RegisterId(BSP_CAN_t bus, uint32_t id, uint8_t queue_size);
int8_t BSP_CAN_RegisterIdParser(BSP_CAN_IdParser_t parser);
int8_t BSP_CAN_GetLatestMessage(BSP_CAN_t bus, uint32_t id,
                                BSP_CAN_Message_t* message);
int8_t BSP_CAN_GetMessage(BSP_CAN_t bus, uint32_t id,
                          BSP_CAN_Message_t* message, uint32_t timeout);
int8_t BSP_CAN_TransmitStdDataFrame(BSP_CAN_t bus,
                                    const BSP_CAN_StdDataFrame_t* frame);
int8_t BSP_CAN_TransmitExtDataFrame(BSP_CAN_t bus,
                                    const BSP_CAN_ExtDataFrame_t* frame);

#ifdef __cplusplus
}
#endif

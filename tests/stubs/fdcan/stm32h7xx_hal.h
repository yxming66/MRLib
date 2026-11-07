#pragma once
#include <stdint.h>
#include <stddef.h>
extern uint32_t fdcan_primask;
static inline uint32_t __get_PRIMASK(void) { return fdcan_primask; }
static inline void __disable_irq(void) { fdcan_primask = 1; }
static inline void __enable_irq(void) { fdcan_primask = 0; }
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t TXBRP, PSR, CCCR, IR; } FDCAN_GlobalTypeDef;
typedef struct {
  uint32_t FrameFormat, TxFifoQueueElmtsNbr, TxBuffersNbr, TxElmtSize;
  uint32_t RxFifo0ElmtsNbr, RxFifo1ElmtsNbr, RxFifo0ElmtSize, RxFifo1ElmtSize;
  uint32_t StdFiltersNbr, ExtFiltersNbr;
} FDCAN_InitTypeDef;
typedef struct {
  FDCAN_GlobalTypeDef* Instance;
  FDCAN_InitTypeDef Init;
  unsigned State;
  uint32_t ErrorCode;
  struct { uint32_t StandardFilterSA, EndAddress; } msgRam;
} FDCAN_HandleTypeDef;
typedef struct {
  uint32_t Identifier, IdType, TxFrameType, DataLength, BitRateSwitch, FDFormat;
  uint32_t ErrorStateIndicator, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct {
  uint32_t FilterIndex, FilterType, FilterConfig, IdType, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
#define SRAMCAN_BASE 0x4000u
#define FDCAN_RX_FIFO0 0u
#define FDCAN_RX_FIFO1 1u
#define FDCAN_FRAME_CLASSIC 0u
#define FDCAN_FRAME_FD_NO_BRS 1u
#define FDCAN_FRAME_FD_BRS 2u
#define FDCAN_STANDARD_ID 0u
#define FDCAN_EXTENDED_ID 1u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_REMOTE_FRAME 1u
#define FDCAN_BRS_OFF 0u
#define FDCAN_BRS_ON 1u
#define FDCAN_CLASSIC_CAN 0u
#define FDCAN_FD_CAN 1u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_NO_TX_EVENTS 0u
#define FDCAN_DATA_BYTES_8 4u
#define FDCAN_DATA_BYTES_64 18u
#define FDCAN_DLC_BYTES_12 9u
#define FDCAN_DLC_BYTES_16 10u
#define FDCAN_DLC_BYTES_20 11u
#define FDCAN_DLC_BYTES_24 12u
#define FDCAN_DLC_BYTES_32 13u
#define FDCAN_DLC_BYTES_48 14u
#define FDCAN_DLC_BYTES_64 15u
#define HAL_FDCAN_STATE_READY 0u
#define HAL_FDCAN_STATE_BUSY 1u
#define HAL_FDCAN_STATE_ERROR 2u
#define FDCAN_PSR_BO 1u
#define FDCAN_CCCR_INIT 1u
#define FDCAN_CCCR_CCE 2u
#define FDCAN_FILTER_MASK 0u
#define FDCAN_FILTER_TO_RXFIFO0 0u
#define FDCAN_FILTER_TO_RXFIFO1 1u
#define FDCAN_ACCEPT_IN_RX_FIFO0 0u
#define FDCAN_ACCEPT_IN_RX_FIFO1 1u
#define FDCAN_FILTER_REMOTE 0u
#define __HAL_FDCAN_CLEAR_FLAG(h,f) ((h)->Instance->IR &= ~(f))
#define FDCAN_IT_ARB_PROTOCOL_ERROR (1u << 0)
#define FDCAN_FLAG_ARB_PROTOCOL_ERROR FDCAN_IT_ARB_PROTOCOL_ERROR
#define FDCAN_IT_BUS_OFF (1u << 1)
#define FDCAN_FLAG_BUS_OFF FDCAN_IT_BUS_OFF
#define FDCAN_IT_DATA_PROTOCOL_ERROR (1u << 2)
#define FDCAN_FLAG_DATA_PROTOCOL_ERROR FDCAN_IT_DATA_PROTOCOL_ERROR
#define FDCAN_IT_ERROR_LOGGING_OVERFLOW (1u << 3)
#define FDCAN_FLAG_ERROR_LOGGING_OVERFLOW FDCAN_IT_ERROR_LOGGING_OVERFLOW
#define FDCAN_IT_ERROR_PASSIVE (1u << 4)
#define FDCAN_FLAG_ERROR_PASSIVE FDCAN_IT_ERROR_PASSIVE
#define FDCAN_IT_ERROR_WARNING (1u << 5)
#define FDCAN_FLAG_ERROR_WARNING FDCAN_IT_ERROR_WARNING
#define FDCAN_IT_RAM_ACCESS_FAILURE (1u << 6)
#define FDCAN_FLAG_RAM_ACCESS_FAILURE FDCAN_IT_RAM_ACCESS_FAILURE
#define FDCAN_IT_RAM_WATCHDOG (1u << 7)
#define FDCAN_FLAG_RAM_WATCHDOG FDCAN_IT_RAM_WATCHDOG
#define FDCAN_IT_RESERVED_ADDRESS_ACCESS (1u << 8)
#define FDCAN_FLAG_RESERVED_ADDRESS_ACCESS FDCAN_IT_RESERVED_ADDRESS_ACCESS
#define FDCAN_IT_RX_FIFO0_FULL (1u << 9)
#define FDCAN_FLAG_RX_FIFO0_FULL FDCAN_IT_RX_FIFO0_FULL
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST (1u << 10)
#define FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST FDCAN_IT_RX_FIFO0_MESSAGE_LOST
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1u << 11)
#define FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE FDCAN_IT_RX_FIFO0_NEW_MESSAGE
#define FDCAN_IT_RX_FIFO1_FULL (1u << 12)
#define FDCAN_FLAG_RX_FIFO1_FULL FDCAN_IT_RX_FIFO1_FULL
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST (1u << 13)
#define FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST FDCAN_IT_RX_FIFO1_MESSAGE_LOST
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE (1u << 14)
#define FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE FDCAN_IT_RX_FIFO1_NEW_MESSAGE
#define FDCAN_IT_TX_ABORT_COMPLETE (1u << 15)
#define FDCAN_FLAG_TX_ABORT_COMPLETE FDCAN_IT_TX_ABORT_COMPLETE
#define FDCAN_IT_TX_COMPLETE (1u << 16)
#define FDCAN_FLAG_TX_COMPLETE FDCAN_IT_TX_COMPLETE
#define FDCAN_IT_TX_FIFO_EMPTY (1u << 17)
#define FDCAN_FLAG_TX_FIFO_EMPTY FDCAN_IT_TX_FIFO_EMPTY
#define HAL_FDCAN_ERROR_NONE 0u
#define HAL_FDCAN_ERROR_NOT_INITIALIZED (1u << 1)
#define HAL_FDCAN_ERROR_NOT_READY (1u << 2)
#define HAL_FDCAN_ERROR_NOT_STARTED (1u << 3)
#define HAL_FDCAN_ERROR_RAM_ACCESS (1u << 4)
#define HAL_FDCAN_ERROR_RAM_WDG (1u << 5)
#define HAL_FDCAN_ERROR_RESERVED_AREA (1u << 6)
#define HAL_FDCAN_ERROR_TIMEOUT (1u << 7)

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*, FDCAN_ProtocolStatusTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef*, FDCAN_ErrorCountersTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef*, uint32_t);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef*, uint32_t);
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef*);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef*, uint32_t);

/* Includes ----------------------------------------------------------------- */
#include "bsp/fdcan.h"
#include "bsp/bsp.h"
#include <cmsis_os2.h>
#include <string.h>

#include "bsp/time.h"

/* Private define ----------------------------------------------------------- */
#define FDCAN_QUEUE_MUTEX_TIMEOUT         100
#define FDCAN_RX_MAX_FRAMES_PER_CALLBACK 64u
#define FDCAN_TX_ABORT_WAIT_COUNT        1000u

/* Private macro ------------------------------------------------------------ */

#define FDCAN_NOTIFY_ERRORS (FDCAN_IT_RAM_ACCESS_FAILURE | FDCAN_IT_ERROR_LOGGING_OVERFLOW | \
    FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF | \
    FDCAN_IT_RAM_WATCHDOG | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | \
    FDCAN_IT_RESERVED_ADDRESS_ACCESS)

typedef struct BSP_FDCAN_QueueNode {
    BSP_FDCAN_t fdcan;
    uint32_t can_id;
    osMessageQueueId_t queue;
    uint8_t queue_size;
  bool latest_only;
  volatile bool latest_valid;
  BSP_FDCAN_Message_t latest_msg;
    struct BSP_FDCAN_QueueNode *next;
} BSP_FDCAN_QueueNode_t;

/* Private variables -------------------------------------------------------- */
static BSP_FDCAN_QueueNode_t *queue_list = NULL;
static osMutexId_t queue_mutex = NULL;
static void (*FDCAN_Callback[BSP_FDCAN_NUM][HAL_FDCAN_CB_NUM])(void);
static bool inited = false;
static bool configured, init_locked;
static BSP_FDCAN_Endpoint_t endpoints[BSP_FDCAN_NUM];
static uint8_t endpoint_count;
static volatile bool active[BSP_FDCAN_NUM];
static BSP_FDCAN_IdParser_t id_parser = NULL;
static BSP_FDCAN_TxQueue_t tx_queues[BSP_FDCAN_NUM];
static const uint8_t fdcan_dlc2len[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
volatile BSP_FDCAN_RxDebug_t g_bsp_fdcan_rx_debug = {0};
static volatile uint32_t fdcan_rx_sequence[BSP_FDCAN_NUM] = {0};
static volatile bool fdcan_recover_pending[BSP_FDCAN_NUM] = {false};
static volatile bool fdcan_tx_drain_active[BSP_FDCAN_NUM] = {false};

/* Private function prototypes ---------------------------------------------- */
static BSP_FDCAN_t FDCAN_Get(FDCAN_HandleTypeDef *hfdcan);
static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNode(BSP_FDCAN_t fdcan, uint32_t can_id);
static int8_t BSP_FDCAN_CreateIdNode(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size, bool latest_only);
static void BSP_FDCAN_DrainRx(BSP_FDCAN_t fdcan, uint32_t fifo);
static BSP_FDCAN_FrameType_t BSP_FDCAN_GetFrameType(FDCAN_RxHeaderTypeDef *header);
static uint32_t BSP_FDCAN_DefaultIdParser(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type);
static void BSP_FDCAN_TxQueueInit(BSP_FDCAN_t fdcan);
static bool BSP_FDCAN_TxQueuePush(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueuePeek(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueuePop(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg);
static bool BSP_FDCAN_TxQueueIsEmpty(BSP_FDCAN_t fdcan);
static void BSP_FDCAN_TxQueueDrain(BSP_FDCAN_t fdcan);
static void BSP_FDCAN_SetTxFifoEmptyNotification(BSP_FDCAN_t fdcan, bool enable);
static uint32_t BSP_FDCAN_NotifyFlags(BSP_FDCAN_t fdcan);
static bool BSP_FDCAN_ErrorNeedsRecovery(FDCAN_HandleTypeDef *hfdcan);
static uint32_t BSP_FDCAN_TxBufferMask(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_FDCAN_AbortPendingTx(FDCAN_HandleTypeDef *hfdcan);
static void BSP_FDCAN_ClearIrFlags(FDCAN_HandleTypeDef *hfdcan);
static int8_t BSP_FDCAN_RecoverIfNeeded(BSP_FDCAN_t fdcan, bool force);

/* Private functions -------------------------------------------------------- */
static BSP_FDCAN_t FDCAN_Get(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (endpoints[i].handle == hfdcan) return (BSP_FDCAN_t)i;
  return BSP_FDCAN_ERR;
}

static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNode(BSP_FDCAN_t fdcan, uint32_t can_id) {
  BSP_FDCAN_QueueNode_t *node = queue_list;
  while (node != NULL) {
    if (node->fdcan == fdcan && node->can_id == can_id) return node;
    node = node->next;
  }
  return NULL;
}

static BSP_FDCAN_QueueNode_t *BSP_FDCAN_FindNodeNoWait(BSP_FDCAN_t fdcan, uint32_t can_id) {
  if (osMutexAcquire(queue_mutex, 0u) != osOK) return NULL;
  BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNode(fdcan, can_id);
  osMutexRelease(queue_mutex);
  return node;
}

static int8_t BSP_FDCAN_CreateIdNode(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size, bool latest_only) {
  if (!BSP_FDCAN_GetHandle(fdcan) || can_id > 0x1fffffffu) return BSP_ERR;
  if (queue_size == 0) queue_size = BSP_FDCAN_DEFAULT_QUEUE_SIZE;
  if (osMutexAcquire(queue_mutex, FDCAN_QUEUE_MUTEX_TIMEOUT) != osOK) return BSP_ERR_TIMEOUT;
  BSP_FDCAN_QueueNode_t *node = queue_list;
  while (node != NULL) {
    if (node->fdcan == fdcan && node->can_id == can_id) {
      osMutexRelease(queue_mutex);
      return BSP_ERR;
    }
    node = node->next;
  }
  
  BSP_FDCAN_QueueNode_t *new_node = (BSP_FDCAN_QueueNode_t *)BSP_Malloc(sizeof(BSP_FDCAN_QueueNode_t));
  if (new_node == NULL) { osMutexRelease(queue_mutex); return BSP_ERR_NULL; }
  memset(new_node, 0, sizeof(BSP_FDCAN_QueueNode_t));
  if (!latest_only) {
    new_node->queue = osMessageQueueNew(queue_size, sizeof(BSP_FDCAN_Message_t), NULL);
    if (new_node->queue == NULL) { BSP_Free(new_node); osMutexRelease(queue_mutex); return BSP_ERR; }
  }
  new_node->fdcan = fdcan;
  new_node->can_id = can_id;
  new_node->queue_size = queue_size;
  new_node->latest_only = latest_only;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  new_node->next = queue_list;
  queue_list = new_node;
  if (primask == 0u) {
    __enable_irq();
  }
  osMutexRelease(queue_mutex);
  return BSP_OK;
}

static BSP_FDCAN_FrameType_t BSP_FDCAN_GetFrameType(FDCAN_RxHeaderTypeDef *header) {
  if (header->RxFrameType == FDCAN_REMOTE_FRAME) {
    return (header->IdType == FDCAN_EXTENDED_ID) ? BSP_FDCAN_FRAME_EXT_REMOTE : BSP_FDCAN_FRAME_STD_REMOTE;
  } else {
    return (header->IdType == FDCAN_EXTENDED_ID) ? BSP_FDCAN_FRAME_EXT_DATA : BSP_FDCAN_FRAME_STD_DATA;
  }
}

static uint32_t BSP_FDCAN_DefaultIdParser(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type) {
  (void)frame_type;
  return original_id;
}

static uint32_t BSP_FDCAN_EncodeDLC(uint8_t dlc) {
  if (dlc <= 8) return dlc;
  if (dlc <= 12) return FDCAN_DLC_BYTES_12;
  if (dlc <= 16) return FDCAN_DLC_BYTES_16;
  if (dlc <= 20) return FDCAN_DLC_BYTES_20;
  if (dlc <= 24) return FDCAN_DLC_BYTES_24;
  if (dlc <= 32) return FDCAN_DLC_BYTES_32;
  if (dlc <= 48) return FDCAN_DLC_BYTES_48;
  return FDCAN_DLC_BYTES_64;
}

static void BSP_FDCAN_TxQueueInit(BSP_FDCAN_t fdcan) {
  if ((unsigned)fdcan >= endpoint_count) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tx_queues[fdcan].head = 0;
  tx_queues[fdcan].tail = 0;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] = 0;
  if (primask == 0u) {
    __enable_irq();
  }
}

static uint32_t BSP_FDCAN_TxQueueDepthUnsafe(const BSP_FDCAN_TxQueue_t *queue) {
  if (queue == NULL) return 0u;
  if (queue->head >= queue->tail) {
    return queue->head - queue->tail;
  }
  return BSP_FDCAN_TX_QUEUE_SIZE - queue->tail + queue->head;
}

static bool BSP_FDCAN_TxQueuePush(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if ((unsigned)fdcan >= endpoint_count || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t next_head = (queue->head + 1) % BSP_FDCAN_TX_QUEUE_SIZE;
  if (next_head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  queue->buffer[queue->head] = *msg;
  queue->head = next_head;
  const uint32_t depth = BSP_FDCAN_TxQueueDepthUnsafe(queue);
  g_bsp_fdcan_rx_debug.tx_queue_push_count[fdcan]++;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] = depth;
  if (depth > g_bsp_fdcan_rx_debug.tx_queue_max_depth[fdcan]) {
    g_bsp_fdcan_rx_debug.tx_queue_max_depth[fdcan] = depth;
  }
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueuePeek(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if ((unsigned)fdcan >= endpoint_count || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (queue->head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  *msg = queue->buffer[queue->tail];
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueuePop(BSP_FDCAN_t fdcan, BSP_FDCAN_TxMessage_t *msg) {
  if ((unsigned)fdcan >= endpoint_count || msg == NULL) return false;
  BSP_FDCAN_TxQueue_t *queue = &tx_queues[fdcan];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (queue->head == queue->tail) {
    if (primask == 0u) {
      __enable_irq();
    }
    return false;
  }
  *msg = queue->buffer[queue->tail];
  queue->tail = (queue->tail + 1) % BSP_FDCAN_TX_QUEUE_SIZE;
  g_bsp_fdcan_rx_debug.tx_queue_pop_count[fdcan]++;
  g_bsp_fdcan_rx_debug.tx_queue_depth[fdcan] =
      BSP_FDCAN_TxQueueDepthUnsafe(queue);
  if (primask == 0u) {
    __enable_irq();
  }
  return true;
}

static bool BSP_FDCAN_TxQueueIsEmpty(BSP_FDCAN_t fdcan) {
  if ((unsigned)fdcan >= endpoint_count) return true;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool is_empty = tx_queues[fdcan].head == tx_queues[fdcan].tail;
  if (primask == 0u) {
    __enable_irq();
  }
  return is_empty;
}

static void BSP_FDCAN_SetTxFifoEmptyNotification(BSP_FDCAN_t fdcan, bool enable) {
  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL) return;
  if (enable) {
    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);
  } else {
    (void)HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
  }
}

static void BSP_FDCAN_TxQueueDrain(BSP_FDCAN_t fdcan) {
  if ((unsigned)fdcan >= endpoint_count) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (fdcan_tx_drain_active[fdcan]) {
    if (primask == 0u) {
      __enable_irq();
    }
    return;
  }
  fdcan_tx_drain_active[fdcan] = true;
  if (primask == 0u) {
    __enable_irq();
  }

  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_BUSY) {
    fdcan_tx_drain_active[fdcan] = false;
    return;
  }

  BSP_FDCAN_TxMessage_t msg;
  while (!BSP_FDCAN_TxQueueIsEmpty(fdcan) &&
         HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u) {
    if (!BSP_FDCAN_TxQueuePeek(fdcan, &msg)) break;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &msg.header, msg.data) != HAL_OK) {
      ++g_bsp_fdcan_rx_debug.tx_add_fail_count[fdcan];
      fdcan_recover_pending[fdcan] = true;
      break;
    }
    (void)BSP_FDCAN_TxQueuePop(fdcan, &msg);
  }

  BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, !BSP_FDCAN_TxQueueIsEmpty(fdcan));
  fdcan_tx_drain_active[fdcan] = false;
}

static uint32_t BSP_FDCAN_TxBufferMask(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return 0u;
  uint32_t count = hfdcan->Init.TxFifoQueueElmtsNbr + hfdcan->Init.TxBuffersNbr;
  if (count >= 32u) return 0xFFFFFFFFu;
  if (count == 0u) return 0u;
  return (1u << count) - 1u;
}

static bool BSP_FDCAN_AbortPendingTx(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return false;
  if (hfdcan->State != HAL_FDCAN_STATE_BUSY) return hfdcan->State == HAL_FDCAN_STATE_READY;
  const uint32_t mask = BSP_FDCAN_TxBufferMask(hfdcan);
  uint32_t pending = hfdcan->Instance->TXBRP & mask;
  if (pending == 0u) return true;

  BSP_FDCAN_t fdcan = FDCAN_Get(hfdcan);
  if (fdcan != BSP_FDCAN_ERR) {
    ++g_bsp_fdcan_rx_debug.tx_abort_count[fdcan];
  }
  if (HAL_FDCAN_AbortTxRequest(hfdcan, pending) != HAL_OK) return false;
  for (uint32_t i = 0u; i < FDCAN_TX_ABORT_WAIT_COUNT; i++) {
    if ((hfdcan->Instance->TXBRP & pending) == 0u) return true;
  }
  return false;
}

static void BSP_FDCAN_ClearIrFlags(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return;
  const uint32_t flags =
      FDCAN_FLAG_TX_COMPLETE |
      FDCAN_FLAG_TX_ABORT_COMPLETE |
      FDCAN_FLAG_TX_FIFO_EMPTY |
      FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST |
      FDCAN_FLAG_RX_FIFO0_FULL |
      FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE |
      FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST |
      FDCAN_FLAG_RX_FIFO1_FULL |
      FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE |
      FDCAN_FLAG_RAM_ACCESS_FAILURE |
      FDCAN_FLAG_ERROR_LOGGING_OVERFLOW |
      FDCAN_FLAG_ERROR_PASSIVE |
      FDCAN_FLAG_ERROR_WARNING |
      FDCAN_FLAG_BUS_OFF |
      FDCAN_FLAG_RAM_WATCHDOG |
      FDCAN_FLAG_ARB_PROTOCOL_ERROR |
      FDCAN_FLAG_DATA_PROTOCOL_ERROR |
      FDCAN_FLAG_RESERVED_ADDRESS_ACCESS;
  __HAL_FDCAN_CLEAR_FLAG(hfdcan, flags);
}

static uint32_t BSP_FDCAN_NotifyFlags(BSP_FDCAN_t fdcan) {
  if ((unsigned)fdcan >= endpoint_count) return 0;
  uint32_t flags = FDCAN_NOTIFY_ERRORS;
  if (endpoints[fdcan].rx_fifos & BSP_FDCAN_RX_FIFO0)
    flags |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
  if (endpoints[fdcan].rx_fifos & BSP_FDCAN_RX_FIFO1)
    flags |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
  return flags;
}

static bool BSP_FDCAN_ErrorNeedsRecovery(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == NULL) return false;
  const uint32_t recover_error_mask =
      HAL_FDCAN_ERROR_TIMEOUT |
      HAL_FDCAN_ERROR_NOT_INITIALIZED |
      HAL_FDCAN_ERROR_NOT_READY |
      HAL_FDCAN_ERROR_NOT_STARTED |
      HAL_FDCAN_ERROR_RAM_ACCESS |
      HAL_FDCAN_ERROR_RAM_WDG |
      HAL_FDCAN_ERROR_RESERVED_AREA;
  return ((hfdcan->ErrorCode & recover_error_mask) != 0u) ||
         ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0u);
}

static int8_t BSP_FDCAN_RecoverIfNeeded(BSP_FDCAN_t fdcan, bool force) {
  if (!inited) return BSP_ERR_INITED;
  if ((unsigned)fdcan >= endpoint_count) return BSP_ERR;
  FDCAN_HandleTypeDef *hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL) return BSP_ERR_NULL;

  FDCAN_ProtocolStatusTypeDef status = {0};
  FDCAN_ErrorCountersTypeDef counters = {0};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &status) == HAL_OK) {
    g_bsp_fdcan_rx_debug.protocol_status[fdcan] = hfdcan->Instance->PSR;
  }
  if (HAL_FDCAN_GetErrorCounters(hfdcan, &counters) == HAL_OK) {
    g_bsp_fdcan_rx_debug.tx_error_count[fdcan] = counters.TxErrorCnt;
    g_bsp_fdcan_rx_debug.rx_error_count[fdcan] = counters.RxErrorCnt;
  }

  const uint32_t psr = hfdcan->Instance->PSR;
  const uint32_t cccr = hfdcan->Instance->CCCR;
  g_bsp_fdcan_rx_debug.protocol_status[fdcan] = psr;

  const bool need_recover = force || fdcan_recover_pending[fdcan] ||
                            (status.BusOff != 0u) ||
                            ((psr & FDCAN_PSR_BO) != 0u) ||
                            BSP_FDCAN_ErrorNeedsRecovery(hfdcan) ||
                            (hfdcan->State == HAL_FDCAN_STATE_READY) ||
                            (hfdcan->State == HAL_FDCAN_STATE_ERROR) ||
                            ((hfdcan->State == HAL_FDCAN_STATE_BUSY) &&
                             ((cccr & FDCAN_CCCR_INIT) != 0u));
  if (!need_recover) return BSP_OK;

  ++g_bsp_fdcan_rx_debug.recover_request_count[fdcan];
  fdcan_recover_pending[fdcan] = false;
  fdcan_tx_drain_active[fdcan] = false;
  BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, false);
  BSP_FDCAN_TxQueueInit(fdcan);
  if (!BSP_FDCAN_AbortPendingTx(hfdcan)) goto failed;
  if (hfdcan->State == HAL_FDCAN_STATE_BUSY && HAL_FDCAN_Stop(hfdcan) != HAL_OK) goto failed;
  if (hfdcan->State != HAL_FDCAN_STATE_READY) goto failed;
  BSP_FDCAN_ClearIrFlags(hfdcan);
  if (HAL_FDCAN_ActivateNotification(hfdcan, BSP_FDCAN_NotifyFlags(fdcan), 0) != HAL_OK) goto failed;
  if (HAL_FDCAN_Start(hfdcan) == HAL_OK) {
    BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, false);
    ++g_bsp_fdcan_rx_debug.recover_success_count[fdcan];
    return BSP_OK;
  }
failed:
  fdcan_recover_pending[fdcan] = true;
  ++g_bsp_fdcan_rx_debug.recover_fail_count[fdcan];
  return BSP_ERR;
}

static void BSP_FDCAN_DrainRx(BSP_FDCAN_t fdcan, uint32_t fifo) {
  const uint8_t bit = fifo == FDCAN_RX_FIFO0 ? BSP_FDCAN_RX_FIFO0 : BSP_FDCAN_RX_FIFO1;
  if ((unsigned)fdcan >= endpoint_count || !active[fdcan] || !(endpoints[fdcan].rx_fifos & bit)) return;
  FDCAN_HandleTypeDef* handle = endpoints[fdcan].handle;
  const uint64_t start_us = BSP_TIME_Get_us();
  // Drain the entry snapshot (hardware FIFO <=64). A fixed smaller cap can
  // strand old frames after HAL clears RFxN, with no new arrival to retrigger it.
  uint32_t budget = HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);
  if (budget > FDCAN_RX_MAX_FRAMES_PER_CALLBACK) budget = FDCAN_RX_MAX_FRAMES_PER_CALLBACK;
  uint32_t handled = 0;
  while (handled < budget && HAL_FDCAN_GetRxFifoFillLevel(handle, fifo)) {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[BSP_FDCAN_MAX_DLC];
    if (HAL_FDCAN_GetRxMessage(handle, fifo, &header, data) != HAL_OK) break;
    ++handled;
    const uint32_t original_id = header.Identifier &
        (header.IdType == FDCAN_STANDARD_ID ? 0x7ffu : 0x1fffffffu);
    const BSP_FDCAN_FrameType_t type = BSP_FDCAN_GetFrameType(&header);
    const uint32_t parsed = BSP_FDCAN_ParseId(original_id, type);
    BSP_FDCAN_QueueNode_t* node = BSP_FDCAN_FindNode(fdcan, parsed);
    if (!node) continue;
    BSP_FDCAN_Message_t msg = {0};
    msg.frame_type = type;
    msg.original_id = original_id;
    msg.parsed_id = parsed;
    msg.sequence = ++fdcan_rx_sequence[fdcan];
    msg.timestamp = (uint32_t)(start_us / 1000u);
    msg.dlc = fdcan_dlc2len[header.DataLength & 0xfu];
    if (header.RxFrameType != FDCAN_REMOTE_FRAME) memcpy(msg.data, data, msg.dlc);
    g_bsp_fdcan_rx_debug.last_rx_sequence[fdcan] = msg.sequence;
    g_bsp_fdcan_rx_debug.last_rx_id[fdcan] = parsed;
    g_bsp_fdcan_rx_debug.last_rx_timestamp_ms[fdcan] = msg.timestamp;
    if (node->latest_only) { node->latest_msg = msg; node->latest_valid = true; }
    else if (node->queue) (void)osMessageQueuePut(node->queue, &msg, 0, 0);
  }
  const uint32_t elapsed = (uint32_t)(BSP_TIME_Get_us() - start_us);
  const bool limited = handled == budget && HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);
  if (fifo == FDCAN_RX_FIFO0) {
    ++g_bsp_fdcan_rx_debug.fifo0_callback_count;
    g_bsp_fdcan_rx_debug.fifo0_frame_count += handled;
    g_bsp_fdcan_rx_debug.fifo0_last_frames = handled;
    g_bsp_fdcan_rx_debug.fifo0_last_us = elapsed;
    if (limited) ++g_bsp_fdcan_rx_debug.fifo0_limited_count;
    if (handled > g_bsp_fdcan_rx_debug.fifo0_max_frames_per_callback) g_bsp_fdcan_rx_debug.fifo0_max_frames_per_callback = handled;
    if (elapsed > g_bsp_fdcan_rx_debug.fifo0_max_us) g_bsp_fdcan_rx_debug.fifo0_max_us = elapsed;
  } else {
    ++g_bsp_fdcan_rx_debug.fifo1_callback_count;
    g_bsp_fdcan_rx_debug.fifo1_frame_count += handled;
    g_bsp_fdcan_rx_debug.fifo1_last_frames = handled;
    g_bsp_fdcan_rx_debug.fifo1_last_us = elapsed;
    if (limited) ++g_bsp_fdcan_rx_debug.fifo1_limited_count;
    if (handled > g_bsp_fdcan_rx_debug.fifo1_max_frames_per_callback) g_bsp_fdcan_rx_debug.fifo1_max_frames_per_callback = handled;
    if (elapsed > g_bsp_fdcan_rx_debug.fifo1_max_us) g_bsp_fdcan_rx_debug.fifo1_max_us = elapsed;
  }
}

/* HAL Callback Stubs (map HAL FDCAN callbacks to user callbacks) */
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {
  (void)TxEventFifoITs;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan]) {
    BSP_FDCAN_TxQueueDrain(bsp_fdcan);
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_EVENT_FIFO_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_EVENT_FIFO_CB]();
  }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan]) {
    BSP_FDCAN_TxQueueDrain(bsp_fdcan);
  }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndex) {
  (void)BufferIndex;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan]) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_COMPLETE_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_COMPLETE_CB]();
  }
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndex) {
  (void)BufferIndex;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan]) {
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_ABORT_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_TX_BUFFER_ABORT_CB]();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if (!(RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL))) return;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan] &&
      (endpoints[bsp_fdcan].rx_fifos & BSP_FDCAN_RX_FIFO0)) {
    BSP_FDCAN_DrainRx(bsp_fdcan, FDCAN_RX_FIFO0);
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO0_MSG_PENDING_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO0_MSG_PENDING_CB]();
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
  if (!(RxFifo1ITs & (FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL))) return;
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan] &&
      (endpoints[bsp_fdcan].rx_fifos & BSP_FDCAN_RX_FIFO1)) {
    BSP_FDCAN_DrainRx(bsp_fdcan, FDCAN_RX_FIFO1);
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB]();
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan != BSP_FDCAN_ERR && active[bsp_fdcan]) {
    ++g_bsp_fdcan_rx_debug.error_callback_count[bsp_fdcan];
    g_bsp_fdcan_rx_debug.last_error_code[bsp_fdcan] = hfdcan->ErrorCode;
    g_bsp_fdcan_rx_debug.protocol_status[bsp_fdcan] = hfdcan->Instance->PSR;
    if (BSP_FDCAN_ErrorNeedsRecovery(hfdcan)) {
      fdcan_recover_pending[bsp_fdcan] = true;
    } else {
      hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
    }
    if (FDCAN_Callback[bsp_fdcan][HAL_FDCAN_ERROR_CB])
      FDCAN_Callback[bsp_fdcan][HAL_FDCAN_ERROR_CB]();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  BSP_FDCAN_t bsp_fdcan = FDCAN_Get(hfdcan);
  if (bsp_fdcan == BSP_FDCAN_ERR || !active[bsp_fdcan]) return;
  ++g_bsp_fdcan_rx_debug.error_status_count[bsp_fdcan];
  g_bsp_fdcan_rx_debug.last_error_status_its[bsp_fdcan] = ErrorStatusITs;
  g_bsp_fdcan_rx_debug.protocol_status[bsp_fdcan] = hfdcan->Instance->PSR;
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0u) {
    ++g_bsp_fdcan_rx_debug.bus_off_count[bsp_fdcan];
    fdcan_recover_pending[bsp_fdcan] = true;
  }
}

/* Exported functions ------------------------------------------------------- */
int8_t BSP_FDCAN_Configure(const BSP_FDCAN_Endpoint_t* table, uint8_t count) {
  if (configured || !table || !count || count > BSP_FDCAN_NUM) return BSP_ERR;
  for (uint8_t i = 0; i < count; ++i) {
    const BSP_FDCAN_Endpoint_t* p = &table[i];
    FDCAN_HandleTypeDef* h = p->handle;
    if (!p->name || !p->name[0] || !h || !h->Instance || h->State != HAL_FDCAN_STATE_READY ||
        !p->rx_fifos || (p->rx_fifos & ~(BSP_FDCAN_RX_FIFO0 | BSP_FDCAN_RX_FIFO1)) ||
        h->Init.RxFifo0ElmtsNbr > 64u || h->Init.RxFifo1ElmtsNbr > 64u ||
        !h->Init.TxFifoQueueElmtsNbr || h->Init.TxFifoQueueElmtsNbr > 32u ||
        h->Init.TxBuffersNbr > 32u || h->Init.TxFifoQueueElmtsNbr + h->Init.TxBuffersNbr > 32u ||
        h->msgRam.StandardFilterSA >= h->msgRam.EndAddress ||
        (h->msgRam.StandardFilterSA & 3u) || (h->msgRam.EndAddress & 3u)) return BSP_ERR;
    const uint32_t required = h->Init.FrameFormat == FDCAN_FRAME_CLASSIC ? FDCAN_DATA_BYTES_8 : FDCAN_DATA_BYTES_64;
    if (h->Init.FrameFormat != FDCAN_FRAME_CLASSIC && h->Init.FrameFormat != FDCAN_FRAME_FD_BRS &&
        h->Init.FrameFormat != FDCAN_FRAME_FD_NO_BRS) return BSP_ERR;
    if (h->Init.TxElmtSize < required ||
        ((p->rx_fifos & BSP_FDCAN_RX_FIFO0) && (!h->Init.RxFifo0ElmtsNbr || h->Init.RxFifo0ElmtSize < required)) ||
        ((p->rx_fifos & BSP_FDCAN_RX_FIFO1) && (!h->Init.RxFifo1ElmtsNbr || h->Init.RxFifo1ElmtSize < required))) return BSP_ERR;
    for (uint8_t j = 0; j < i; ++j) {
      FDCAN_HandleTypeDef* other = table[j].handle;
      if (!strcmp(p->name, table[j].name) || h->Instance == other->Instance ||
          (h->msgRam.StandardFilterSA < other->msgRam.EndAddress &&
           other->msgRam.StandardFilterSA < h->msgRam.EndAddress)) return BSP_ERR;
    }
  }
  memcpy(endpoints, table, count * sizeof(*table));
  endpoint_count = count;
  configured = true;
  return BSP_OK;
}
BSP_FDCAN_t BSP_FDCAN_FindEndpoint(const char* name) {
  if (name)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (!strcmp(name, endpoints[i].name)) return (BSP_FDCAN_t)i;
  return BSP_FDCAN_ERR;
}
FDCAN_HandleTypeDef* BSP_FDCAN_GetHandle(BSP_FDCAN_t fdcan) {
  return (unsigned)fdcan < endpoint_count ? endpoints[fdcan].handle : NULL;
}
int8_t BSP_FDCAN_Init(void) {
  if (inited || init_locked) return BSP_ERR_INITED;
  if (!configured || osKernelGetState() != osKernelRunning) return BSP_ERR;
  if (!queue_mutex) queue_mutex = osMutexNew(NULL);
  if (!queue_mutex) return BSP_ERR;
  id_parser = BSP_FDCAN_DefaultIdParser;
  for (uint8_t i = 0; i < endpoint_count; ++i) {
    BSP_FDCAN_TxQueueInit((BSP_FDCAN_t)i);
    FDCAN_HandleTypeDef* h = endpoints[i].handle;
    if (h->State != HAL_FDCAN_STATE_READY) goto fail;
    active[i] = true; // Software is ready before hardware can raise an IRQ.
    if (HAL_FDCAN_ActivateNotification(h, BSP_FDCAN_NotifyFlags((BSP_FDCAN_t)i), 0) != HAL_OK ||
        HAL_FDCAN_Start(h) != HAL_OK) goto fail;
  }
  inited = true;
  return BSP_OK;
fail:
  for (uint8_t i = 0; i < endpoint_count; ++i) {
    if (!active[i]) continue;
    FDCAN_HandleTypeDef* h = endpoints[i].handle;
    if (HAL_FDCAN_DeactivateNotification(h, BSP_FDCAN_NotifyFlags((BSP_FDCAN_t)i) | FDCAN_IT_TX_FIFO_EMPTY) != HAL_OK)
      init_locked = true;
    if (h->State != HAL_FDCAN_STATE_READY && HAL_FDCAN_Stop(h) != HAL_OK) init_locked = true;
    active[i] = false;
  }
  return BSP_ERR;
}

int8_t BSP_FDCAN_RegisterCallback(BSP_FDCAN_t fdcan, BSP_FDCAN_Callback_t type, void (*callback)(void)) {
  if (!inited || !BSP_FDCAN_GetHandle(fdcan) || !callback || (unsigned)type >= HAL_FDCAN_CB_NUM) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  const bool free = FDCAN_Callback[fdcan][type] == NULL;
  if (free) FDCAN_Callback[fdcan][type] = callback;
  if (!token) __enable_irq();
  return free ? BSP_OK : BSP_ERR;
}
int8_t BSP_FDCAN_UnregisterCallback(BSP_FDCAN_t fdcan, BSP_FDCAN_Callback_t type, void (*callback)(void)) {
  if (!inited || !BSP_FDCAN_GetHandle(fdcan) || !callback || (unsigned)type >= HAL_FDCAN_CB_NUM) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  const bool owned = FDCAN_Callback[fdcan][type] == callback;
  if (owned) FDCAN_Callback[fdcan][type] = NULL;
  if (!token) __enable_irq();
  return owned ? BSP_OK : BSP_ERR;
}

int8_t BSP_FDCAN_Transmit(BSP_FDCAN_t fdcan, BSP_FDCAN_Format_t format, uint32_t id,
                          const uint8_t* data, uint8_t dlc) {
  if (!inited) return BSP_ERR_INITED;
  if ((unsigned)fdcan >= endpoint_count) return BSP_ERR;
  if (data == NULL && format != BSP_FDCAN_FORMAT_STD_REMOTE &&
      format != BSP_FDCAN_FORMAT_EXT_REMOTE)
    return BSP_ERR_NULL;
  if (dlc > BSP_FDCAN_MAX_DLC) return BSP_ERR;
  FDCAN_HandleTypeDef* hfdcan = BSP_FDCAN_GetHandle(fdcan);
  if (hfdcan == NULL) return BSP_ERR_NULL;
  const bool standard =
      format == BSP_FDCAN_FORMAT_STD_DATA || format == BSP_FDCAN_FORMAT_STD_REMOTE;
  const bool extended =
      format == BSP_FDCAN_FORMAT_EXT_DATA || format == BSP_FDCAN_FORMAT_EXT_REMOTE;
  const bool remote =
      format == BSP_FDCAN_FORMAT_STD_REMOTE || format == BSP_FDCAN_FORMAT_EXT_REMOTE;
  if ((!standard && !extended) || id > (standard ? 0x7ffu : 0x1fffffffu) ||
      ((remote || hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC) && dlc > 8u))
    return BSP_ERR;
  if (BSP_FDCAN_RecoverIfNeeded(fdcan, false) != BSP_OK) return BSP_ERR;

  BSP_FDCAN_TxMessage_t tx_msg = {0};
  switch (format) {
      case BSP_FDCAN_FORMAT_STD_DATA:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_STANDARD_ID;
          tx_msg.header.TxFrameType = FDCAN_DATA_FRAME;
          break;
      case BSP_FDCAN_FORMAT_EXT_DATA:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_EXTENDED_ID;
          tx_msg.header.TxFrameType = FDCAN_DATA_FRAME;
          break;
      case BSP_FDCAN_FORMAT_STD_REMOTE:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_STANDARD_ID;
          tx_msg.header.TxFrameType = FDCAN_REMOTE_FRAME;
          break;
      case BSP_FDCAN_FORMAT_EXT_REMOTE:
          tx_msg.header.Identifier = id;
          tx_msg.header.IdType = FDCAN_EXTENDED_ID;
          tx_msg.header.TxFrameType = FDCAN_REMOTE_FRAME;
          break;
      default:
          return BSP_ERR;
  }
  switch (hfdcan->Init.FrameFormat) {
    case FDCAN_FRAME_FD_BRS:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_ON;
      tx_msg.header.FDFormat = FDCAN_FD_CAN;
      break;
    case FDCAN_FRAME_FD_NO_BRS:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_msg.header.FDFormat = FDCAN_FD_CAN;
      break;
    case FDCAN_FRAME_CLASSIC:
    default:
      tx_msg.header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_msg.header.FDFormat = FDCAN_CLASSIC_CAN;
      break;
  }
  if (remote) {
    tx_msg.header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_msg.header.FDFormat = FDCAN_CLASSIC_CAN;
  }
        tx_msg.header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_msg.header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_msg.header.MessageMarker = 0x00;
  tx_msg.header.DataLength = BSP_FDCAN_EncodeDLC(dlc);   

  memset(tx_msg.data, 0, dlc);
  if (data != NULL && dlc > 0) {memcpy(tx_msg.data, data, dlc);}

  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_msg.header, tx_msg.data) ==
        HAL_OK) {
      ++g_bsp_fdcan_rx_debug.tx_direct_count[fdcan];
      return BSP_OK;
    }
    ++g_bsp_fdcan_rx_debug.tx_add_fail_count[fdcan];
    fdcan_recover_pending[fdcan] = true;
  }
  if (BSP_FDCAN_TxQueuePush(fdcan, &tx_msg)) {
    BSP_FDCAN_SetTxFifoEmptyNotification(fdcan, true);
    BSP_FDCAN_TxQueueDrain(fdcan);
    return BSP_OK;
  }
  ++g_bsp_fdcan_rx_debug.tx_queue_full_count[fdcan];
  return BSP_ERR;
}

int8_t BSP_FDCAN_TransmitStdDataFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_StdDataFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    return BSP_FDCAN_Transmit(fdcan, BSP_FDCAN_FORMAT_STD_DATA, frame->id, frame->data, frame->dlc);
}

int8_t BSP_FDCAN_TransmitExtDataFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_ExtDataFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    return BSP_FDCAN_Transmit(fdcan, BSP_FDCAN_FORMAT_EXT_DATA, frame->id, frame->data, frame->dlc);
}

int8_t BSP_FDCAN_TransmitRemoteFrame(BSP_FDCAN_t fdcan, BSP_FDCAN_RemoteFrame_t *frame) {
    if (frame == NULL) return BSP_ERR_NULL;
    BSP_FDCAN_Format_t format = frame->is_extended ? BSP_FDCAN_FORMAT_EXT_REMOTE : BSP_FDCAN_FORMAT_STD_REMOTE;
    return BSP_FDCAN_Transmit(fdcan, format, frame->id, NULL, frame->dlc);
}

int8_t BSP_FDCAN_RegisterId(BSP_FDCAN_t fdcan, uint32_t can_id, uint8_t queue_size) {
    if (!inited) return BSP_ERR_INITED;
    return BSP_FDCAN_CreateIdNode(fdcan, can_id, queue_size, false);
}

int8_t BSP_FDCAN_UnregisterId(BSP_FDCAN_t fdcan, uint32_t can_id) {
  if (!inited) return BSP_ERR_INITED;
  if ((unsigned)fdcan >= endpoint_count) return BSP_ERR;
  if (osMutexAcquire(queue_mutex, FDCAN_QUEUE_MUTEX_TIMEOUT) != osOK) return BSP_ERR_TIMEOUT;
  BSP_FDCAN_QueueNode_t** slot = &queue_list;
  while (*slot && ((*slot)->fdcan != fdcan || (*slot)->can_id != can_id)) slot = &(*slot)->next;
  BSP_FDCAN_QueueNode_t* node = *slot;
  if (!node) {
    osMutexRelease(queue_mutex);
    return BSP_ERR_NO_DEV;
  }
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  *slot = node->next;
  if (token == 0u) __enable_irq();
  if (node->queue && osMessageQueueDelete(node->queue) != osOK) {
    const uint32_t restore = __get_PRIMASK();
    __disable_irq();
    node->next = *slot;
    *slot = node;
    if (restore == 0u) __enable_irq();
    osMutexRelease(queue_mutex);
    return BSP_ERR;
  }
  BSP_Free(node);
  osMutexRelease(queue_mutex);
  return BSP_OK;
}

int8_t BSP_FDCAN_RegisterLatestId(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return BSP_ERR_INITED;
    return BSP_FDCAN_CreateIdNode(fdcan, can_id, 1u, true);
}

int8_t BSP_FDCAN_GetMessage(BSP_FDCAN_t fdcan, uint32_t can_id, BSP_FDCAN_Message_t *msg, uint32_t timeout) {
    if (!inited) return BSP_ERR_INITED;
    if (msg == NULL) return BSP_ERR_NULL;
    (void)BSP_FDCAN_RecoverIfNeeded(fdcan, false);
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return BSP_ERR_NO_DEV;
    osStatus_t res = osMessageQueueGet(queue, msg, NULL, timeout);
    return (res == osOK) ? BSP_OK : BSP_ERR;
}

int8_t BSP_FDCAN_GetLatestMessage(BSP_FDCAN_t fdcan, uint32_t can_id, BSP_FDCAN_Message_t *msg) {
    if (!inited) return BSP_ERR_INITED;
    if (msg == NULL) return BSP_ERR_NULL;
  (void)BSP_FDCAN_RecoverIfNeeded(fdcan, false);
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    if (node == NULL) return BSP_ERR_NO_DEV;
    if (node->latest_only) {
        const uint32_t token = __get_PRIMASK();
        __disable_irq();
        const bool valid = node->latest_valid;
        if (valid) *msg = node->latest_msg;
        if (!token) __enable_irq();
        return valid ? BSP_OK : BSP_ERR;
    }
    osMessageQueueId_t queue = node->queue;
    if (queue == NULL) return BSP_ERR_NO_DEV;

    BSP_FDCAN_Message_t latest;
    bool has_latest = false;
    while (osMessageQueueGet(queue, &latest, NULL, BSP_FDCAN_TIMEOUT_IMMEDIATE) == osOK) {
        has_latest = true;
    }
    if (!has_latest) return BSP_ERR;
    *msg = latest;
    return BSP_OK;
}

  int8_t BSP_FDCAN_Recover(BSP_FDCAN_t fdcan) {
    return BSP_FDCAN_RecoverIfNeeded(fdcan, true);
  }

int32_t BSP_FDCAN_GetQueueCount(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return -1;
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return -1;
    return (int32_t)osMessageQueueGetCount(queue);
}

int8_t BSP_FDCAN_FlushQueue(BSP_FDCAN_t fdcan, uint32_t can_id) {
    if (!inited) return BSP_ERR_INITED;
    BSP_FDCAN_QueueNode_t *node = BSP_FDCAN_FindNodeNoWait(fdcan, can_id);
    if (node != NULL && node->latest_only) {
        const uint32_t token = __get_PRIMASK();
        __disable_irq();
        node->latest_valid = false;
        if (!token) __enable_irq();
        return BSP_OK;
    }
    osMessageQueueId_t queue = (node != NULL) ? node->queue : NULL;
    if (queue == NULL) return BSP_ERR_NO_DEV;
    BSP_FDCAN_Message_t tmp;
    while (osMessageQueueGet(queue, &tmp, NULL, BSP_FDCAN_TIMEOUT_IMMEDIATE) == osOK) { }
    return BSP_OK;
}

int8_t BSP_FDCAN_RegisterIdParser(BSP_FDCAN_IdParser_t parser) {
    if (!inited) return BSP_ERR_INITED;
    if (parser == NULL) return BSP_ERR_NULL;
    id_parser = parser;
    return BSP_OK;
}

uint32_t BSP_FDCAN_ParseId(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type) {
    if (id_parser != NULL) return id_parser(original_id, frame_type);
    return BSP_FDCAN_DefaultIdParser(original_id, frame_type);
}
/* */


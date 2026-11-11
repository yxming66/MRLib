#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsp/bsp.h"
#include "bsp/mm.h"

/* USER INCLUDE BEGIN */
#include "stm32h7xx_hal.h"
/* USER INCLUDE END */

/* Exported constants ------------------------------------------------------- */
#define BSP_FDCAN_MAX_DLC 64
#define BSP_FDCAN_DEFAULT_QUEUE_SIZE 10
#define BSP_FDCAN_TIMEOUT_IMMEDIATE 0
#define BSP_FDCAN_TIMEOUT_FOREVER osWaitForever
#define BSP_FDCAN_TX_QUEUE_SIZE 32
/* Exported macro ----------------------------------------------------------- */
// Neutral slots in board registration order, NOT physical controller numbers.
typedef enum { BSP_FDCAN_1 = 0, BSP_FDCAN_2, BSP_FDCAN_3,
               BSP_FDCAN_NUM, BSP_FDCAN_ERR } BSP_FDCAN_t;
enum { BSP_FDCAN_RX_FIFO0 = 1u, BSP_FDCAN_RX_FIFO1 = 2u };
typedef struct {
  const char* name;
  FDCAN_HandleTypeDef* handle;
  uint8_t rx_fifos; // Bitmask of BSP_FDCAN_RX_FIFO0/1, matching board filters.
} BSP_FDCAN_Endpoint_t;

// Startup-only, after Cube HAL init, before controller Start. Board must finish
// filters before BSP_FDCAN_Init; Configure may precede filter RAM writes.
// Copies the table; names/handles persist. No runtime rebinding or hardware init.
// Board owns clocks, timing, RAM capacity/layout, filters and NVIC configuration.
// 启动时登记板级控制器映射；复制表项，不初始化硬件，不支持运行中重绑定。
int8_t BSP_FDCAN_Configure(const BSP_FDCAN_Endpoint_t* endpoints, uint8_t count);
// 按登记名称查找控制器端点，找不到时返回 BSP_FDCAN_ERR。
BSP_FDCAN_t BSP_FDCAN_FindEndpoint(const char* name);

typedef enum {
  HAL_FDCAN_TX_EVENT_FIFO_CB,
  HAL_FDCAN_TX_BUFFER_COMPLETE_CB,
  HAL_FDCAN_TX_BUFFER_ABORT_CB,
  HAL_FDCAN_RX_FIFO0_MSG_PENDING_CB,
  HAL_FDCAN_RX_FIFO0_FULL_CB,
  HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB,
  HAL_FDCAN_RX_FIFO1_FULL_CB,
  HAL_FDCAN_ERROR_CB,
  HAL_FDCAN_CB_NUM,
} BSP_FDCAN_Callback_t;

typedef enum {
  BSP_FDCAN_FORMAT_STD_DATA,
  BSP_FDCAN_FORMAT_EXT_DATA,
  BSP_FDCAN_FORMAT_STD_REMOTE,
  BSP_FDCAN_FORMAT_EXT_REMOTE,
} BSP_FDCAN_Format_t;

typedef enum {
  BSP_FDCAN_FRAME_STD_DATA,
  BSP_FDCAN_FRAME_EXT_DATA,
  BSP_FDCAN_FRAME_STD_REMOTE,
  BSP_FDCAN_FRAME_EXT_REMOTE,
} BSP_FDCAN_FrameType_t;

typedef struct {
  BSP_FDCAN_FrameType_t frame_type;
  uint32_t original_id;
  uint32_t parsed_id;
  uint32_t sequence;
  uint8_t dlc;
  uint8_t data[BSP_FDCAN_MAX_DLC];
  uint32_t timestamp;
} BSP_FDCAN_Message_t;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[BSP_FDCAN_MAX_DLC];
} BSP_FDCAN_StdDataFrame_t;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[BSP_FDCAN_MAX_DLC];
} BSP_FDCAN_ExtDataFrame_t;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  bool is_extended;
} BSP_FDCAN_RemoteFrame_t;

typedef uint32_t (*BSP_FDCAN_IdParser_t)(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type);

typedef struct {
  FDCAN_TxHeaderTypeDef header; /* HAL FDCAN header type */
  uint8_t data[BSP_FDCAN_MAX_DLC];
} BSP_FDCAN_TxMessage_t;

typedef struct {
  BSP_FDCAN_TxMessage_t buffer[BSP_FDCAN_TX_QUEUE_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} BSP_FDCAN_TxQueue_t;

typedef struct {
  volatile uint32_t fifo0_callback_count;
  volatile uint32_t fifo1_callback_count;
  volatile uint32_t fifo0_frame_count;
  volatile uint32_t fifo1_frame_count;
  volatile uint32_t fifo0_limited_count;
  volatile uint32_t fifo1_limited_count;
  volatile uint32_t fifo0_max_frames_per_callback;
  volatile uint32_t fifo1_max_frames_per_callback;
  volatile uint32_t fifo0_last_frames;
  volatile uint32_t fifo1_last_frames;
  volatile uint32_t fifo0_last_us;
  volatile uint32_t fifo1_last_us;
  volatile uint32_t fifo0_max_us;
  volatile uint32_t fifo1_max_us;
  volatile uint32_t last_rx_sequence[BSP_FDCAN_NUM];
  volatile uint32_t last_rx_id[BSP_FDCAN_NUM];
  volatile uint32_t last_rx_timestamp_ms[BSP_FDCAN_NUM];
  volatile uint32_t error_status_count[BSP_FDCAN_NUM];
  volatile uint32_t bus_off_count[BSP_FDCAN_NUM];
  volatile uint32_t recover_request_count[BSP_FDCAN_NUM];
  volatile uint32_t recover_success_count[BSP_FDCAN_NUM];
  volatile uint32_t recover_fail_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_error_count[BSP_FDCAN_NUM];
  volatile uint32_t rx_error_count[BSP_FDCAN_NUM];
  volatile uint32_t protocol_status[BSP_FDCAN_NUM];
  volatile uint32_t error_callback_count[BSP_FDCAN_NUM];
  volatile uint32_t last_error_code[BSP_FDCAN_NUM];
  volatile uint32_t last_error_status_its[BSP_FDCAN_NUM];
  volatile uint32_t tx_add_fail_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_abort_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_queue_full_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_direct_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_queue_push_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_queue_pop_count[BSP_FDCAN_NUM];
  volatile uint32_t tx_queue_depth[BSP_FDCAN_NUM];
  volatile uint32_t tx_queue_max_depth[BSP_FDCAN_NUM];
} BSP_FDCAN_RxDebug_t;

extern volatile BSP_FDCAN_RxDebug_t g_bsp_fdcan_rx_debug;

/* Exported functions prototypes -------------------------------------------- */
// Task-only, once RTOS is running. Prepares software state, then starts only the
// registered controllers. Failure stops this attempt's controllers; failed HAL
// rollback locks initialization until reset. Never reinitializes shared RAM.
// 在 RTOS 任务中初始化软件状态并启动已登记控制器；不重新初始化共享消息 RAM。
int8_t BSP_FDCAN_Init(void);
// 取得端点对应的 HAL 句柄，无效端点返回 NULL。
FDCAN_HandleTypeDef* BSP_FDCAN_GetHandle(BSP_FDCAN_t can);
// 追加指定事件的观察回调，不替换 BSP 接收处理或已有回调。
int8_t BSP_FDCAN_RegisterCallback(BSP_FDCAN_t can, BSP_FDCAN_Callback_t type,
                                  void (*callback)(void));
// Observer callbacks never replace BSP reception or another callback.
// 移除指定事件中匹配的观察回调，不影响其他回调。
int8_t BSP_FDCAN_UnregisterCallback(BSP_FDCAN_t can, BSP_FDCAN_Callback_t type,
                                    void (*callback)(void));
// 按帧格式发送或排队一帧；dlc 为字节数，成功不代表已获总线应答。
int8_t BSP_FDCAN_Transmit(BSP_FDCAN_t can, BSP_FDCAN_Format_t format, uint32_t id,
                          const uint8_t* data, uint8_t dlc);
// 发送标准 ID 数据帧，忙时使用软件发送队列。
int8_t BSP_FDCAN_TransmitStdDataFrame(BSP_FDCAN_t can, BSP_FDCAN_StdDataFrame_t* frame);
// 发送扩展 ID 数据帧，忙时使用软件发送队列。
int8_t BSP_FDCAN_TransmitExtDataFrame(BSP_FDCAN_t can, BSP_FDCAN_ExtDataFrame_t* frame);
// 发送标准或扩展 ID 的远程请求帧。
int8_t BSP_FDCAN_TransmitRemoteFrame(BSP_FDCAN_t can, BSP_FDCAN_RemoteFrame_t* frame);
// 为接收 ID 登记 FIFO 队列订阅；queue_size 为 0 时使用默认深度。
int8_t BSP_FDCAN_RegisterId(BSP_FDCAN_t can, uint32_t can_id, uint8_t queue_size);
/* Task-only, with all application readers of this ID stopped. ISR delivery is
 * unlinked under IRQ protection before deleting its queue. */
// 在任务中撤销 ID 订阅并释放队列；调用前须停止该 ID 的所有应用读取者。
int8_t BSP_FDCAN_UnregisterId(BSP_FDCAN_t can, uint32_t can_id);
// 为接收 ID 登记最新值订阅；新帧覆盖旧帧，不创建逐帧队列。
int8_t BSP_FDCAN_RegisterLatestId(BSP_FDCAN_t can, uint32_t can_id);
// 从普通订阅队列取走最早一帧，等待时间单位为 RTOS tick。
int8_t BSP_FDCAN_GetMessage(BSP_FDCAN_t can, uint32_t can_id, BSP_FDCAN_Message_t* msg,
                            uint32_t timeout);
// 最新值订阅读取快照；普通队列订阅则排空队列并取最后一帧。
int8_t BSP_FDCAN_GetLatestMessage(BSP_FDCAN_t can, uint32_t can_id, BSP_FDCAN_Message_t* msg);
// 尝试恢复指定控制器通信，不执行整套 HAL 或共享 RAM 重新初始化。
int8_t BSP_FDCAN_Recover(BSP_FDCAN_t can);
// 查询普通订阅队列的待取帧数；无效或最新值订阅返回 -1。
int32_t BSP_FDCAN_GetQueueCount(BSP_FDCAN_t can, uint32_t can_id);
// 清空普通订阅队列，或使最新值订阅的已有快照失效。
int8_t BSP_FDCAN_FlushQueue(BSP_FDCAN_t can, uint32_t can_id);
// 初始化后设置全局 ID 解析器；再次设置会替换原解析器。
int8_t BSP_FDCAN_RegisterIdParser(BSP_FDCAN_IdParser_t parser);
// 用已设置解析器或默认规则，将原始 ID 与帧类型转换为订阅键。
uint32_t BSP_FDCAN_ParseId(uint32_t original_id, BSP_FDCAN_FrameType_t frame_type);
#ifdef __cplusplus
}
#endif

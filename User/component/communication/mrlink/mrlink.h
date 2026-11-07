#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Wire: header[2], payload length u8, cmd u8, payload, optional CRC16 LE.
 * CRC is reflected 0x1021, init FFFF, xorout 0. Default header is ASCII MR.
 * No HAL/RTOS, heap allocation, platform globals or native-struct wire assumption.
 */
/* Return codes / 错误码 */
#define MRLINK_OK            (0)
#define MRLINK_ERR           (-1)   /**< 通用错误 (如无完整帧) */
#define MRLINK_ERR_NULL      (-2)   /**< 空指针 / Null pointer */
#define MRLINK_ERR_OVERSIZE  (-3)   /**< payload 超 max_payload_size */
#define MRLINK_ERR_CRC       (-4)   /**< CRC 校验失败 / CRC mismatch */
#define MRLINK_ERR_HEADER    (-5)   /**< 帧头不匹配 / Header mismatch */
#define MRLINK_ERR_ARGS      (-6)   /**< 参数非法 / Invalid args */
#define MRLINK_ERR_BUF_FULL  (-7)   /**< 内部流缓冲满 / Stream buf full */

/** Maximum work performed by one MrLink_Dispatch call. */
#ifndef MRLINK_DISPATCH_MAX_FRAMES
#define MRLINK_DISPATCH_MAX_FRAMES (16u)
#endif
#ifndef MRLINK_DISPATCH_MAX_BYTES
#define MRLINK_DISPATCH_MAX_BYTES (1024u)
#endif

/* Default frame header (project identifier "MR") */
#define MRLINK_DEFAULT_HEADER_0 (0x4D)  /**< 'M' */
#define MRLINK_DEFAULT_HEADER_1 (0x52)  /**< 'R' */
#define MRLINK_HEADER_0      MRLINK_DEFAULT_HEADER_0
#define MRLINK_HEADER_1      MRLINK_DEFAULT_HEADER_1
#define MRLINK_HEADER_LEN    (2u)

/* Field lengths */
#define MRLINK_LEN_FIELD_LEN (1u)
#define MRLINK_CMD_FIELD_LEN (1u)
#define MRLINK_CRC_LEN       (2u)

/* Frame total length with CRC:    2 + 1 + 1 + payload + 2 = 6 + payload */
/* Frame total length without CRC: 2 + 1 + 1 + payload     = 4 + payload */

/* Defaults */
#define MRLINK_MAX_PAYLOAD_DEFAULT (64u)

/** Handler dispatch table size / handler 表大小.
 *  64 槽
 *  若需更大表，修改此宏并重新编译。 */
#define MRLINK_MAX_HANDLERS (64u)

/** 单帧最大长度 (header 2 + len 1 + cmd 1 + max_payload + crc 2)
 *  默认 max_payload=64, use_crc16=true 时 = 70. 当前 parser 与内部 frame_buf
 *  以此编译期宏为上限；未来支持分片或更大 payload 时，需同步重审
 *  MrLink_t storage、Parse/typed handler 临时缓冲和各 channel max_frame_size。 */
#define MRLINK_MAX_FRAME_SIZE \
  (MRLINK_HEADER_LEN + MRLINK_LEN_FIELD_LEN + \
   MRLINK_CMD_FIELD_LEN + MRLINK_MAX_PAYLOAD_DEFAULT + \
   MRLINK_CRC_LEN)


#define MRLINK_ERR_BUSY (-8)

typedef struct MrLink MrLink_t;
typedef struct {
  uint16_t max_payload_size;     /**< 单帧 payload 上限，默认 64 */
  bool use_crc16;                /**< true=启用 mrlink CRC16，算法见 User/component/crc16.c；默认 true */
  uint8_t header_0;              /**< 帧头第 1 字节；0/0 表示使用默认 "MR" */
  uint8_t header_1;              /**< 帧头第 2 字节；0/0 表示使用默认 "MR" */
} MrLink_Config_t;

/**
 * @brief 运行统计 / Runtime stats.
 *
 * 通过 GetStatsSnapshot 在任务上下文获取一致快照。
 */
typedef struct {
  uint32_t frame_rx_ok;            /**< 成功解析的帧数 */
  uint32_t frame_rx_crc_err;       /**< CRC 错 (use_crc16=true 时统计) */
  uint32_t frame_rx_oversize;      /**< length 字段超 max_payload_size */
  uint32_t frame_rx_header_skip;   /**< 跳过非帧头字节总数 */
  uint32_t frame_rx_truncated;     /**< 收不完整帧 (流缓冲不够) */
  uint32_t frame_rx_unknown_cmd;   /**< 收到但无 handler 的 cmd 帧数 */
  uint32_t frame_rx_size_mismatch; /**< typed handler 期望长度不匹配 (comm_rx_size_mismatch) */
  uint32_t frame_tx_ok;            /**< 成功 build 的帧数 */
  uint32_t feed_bytes_total;       /**< PushBytes 喂入字节总数 */
  uint32_t feed_bytes_dropped;     /**< RX ring buffer 满导致丢弃的字节数 */
  uint32_t feed_calls;             /**< PushBytes 调用次数 */
} MrLink_Stats_t;

/**
 * @brief 错误事件类型 / Error event code.
 */
typedef enum {
  MRLINK_ERROR_HEADER_SKIP = 1,     /**< 跳过非帧头字节 */
  MRLINK_ERROR_OVERSIZE = 2,        /**< payload length 超配置上限 */
  MRLINK_ERROR_TRUNCATED = 3,       /**< 当前缓冲不足一帧 */
  MRLINK_ERROR_CRC = 4,             /**< CRC 校验失败 */
  MRLINK_ERROR_UNKNOWN_CMD = 5,     /**< 无 handler 的 cmd */
  MRLINK_ERROR_SIZE_MISMATCH = 6,   /**< typed payload 长度不匹配 */
  MRLINK_ERROR_RX_DROPPED = 7,      /**< RX ring buffer 满导致丢字节 */
} MrLink_ErrorCode_t;

/**
 * @brief 错误事件信息 / Error event details.
 */
typedef struct {
  MrLink_ErrorCode_t code;
  uint8_t cmd;
  uint16_t payload_len;
  uint16_t expected_size;
  uint16_t available;
  uint16_t dropped_bytes;
  uint16_t crc_received;
  uint16_t crc_calculated;
} MrLink_ErrorInfo_t;

typedef void (*MrLink_ErrorHandler_t)(const MrLink_ErrorInfo_t *info,
                                      void *ctx);

/**
 * @brief 帧回调 / Frame handler callback.
 *
 * 每次成功解析出一帧 (Dispatch 或 Parse 触发) 时调用一次。
 * 回调内应**短小可重入**，不要做长阻塞。
 *
 * @param cmd          帧命令字
 * @param payload      指向库内临时 payload 缓冲 (本次调用内有效)
 * @param payload_len  payload 字节数
 * @param ctx          RegisterHandler 时传入的上下文
 */
typedef int8_t (*MrLink_FrameHandler_t)(uint8_t cmd,
                                            const uint8_t *payload,
                                            uint16_t payload_len,
                                            void *ctx);

/** Length-checked bytes, NOT a live T object. Decode fields explicitly or memcpy
 * into an existing trivially-copyable object with a known legacy ABI. */
typedef int8_t (*MrLink_TypedHandler_t)(uint8_t cmd,
                                            const void *typed_data,
                                            uint16_t typed_size,
                                            void *ctx);


/* Optional paired synchronization hooks. NULL/NULL means a single execution
 * context. For task + ISR input install IRQ-save/restore hooks before starting
 * reception. Hooks protect only bounded ring copies/counters, NEVER callbacks.
 * Init/Reset/SetSynchronization require producer and consumer to be quiescent.
 * Exactly one producer and one task consumer; registration/build/cache use that
 * task. No NMI, multicore or desktop-thread guarantees without suitable hooks.
 */
typedef uintptr_t (*MrLink_Enter_t)(void *ctx);
typedef void (*MrLink_Exit_t)(void *ctx, uintptr_t token);
typedef struct {
  MrLink_Enter_t enter;
  MrLink_Exit_t exit;
  void *ctx;
} MrLink_Synchronization_t;

typedef struct {
  bool used;
  uint8_t cmd;
  bool typed;
  uint16_t expected_size;
  union { MrLink_FrameHandler_t raw; MrLink_TypedHandler_t typed; } handler;
  void *ctx;
} MrLink_HandlerEntry_t;

/* One real layout shared by C and C++. Internal fields: access via the API only.
 * Instances/storage must not be copied or moved during use. */
struct MrLink {
  MrLink_Config_t cfg;
  MrLink_Synchronization_t sync;
  uint8_t *rx_buf;
  uint16_t rx_buf_size;
  uint16_t rx_head, rx_tail;
  uint8_t *tx_buf;
  uint16_t tx_buf_size;
  uint8_t frame_buf[MRLINK_MAX_FRAME_SIZE];
  MrLink_HandlerEntry_t handlers[MRLINK_MAX_HANDLERS];
  MrLink_Stats_t stats;
  MrLink_ErrorHandler_t error_handler;
  void *error_ctx;
  uint32_t pending_rx_dropped;
  bool processing;
};

/* Failed Init leaves the previous instance untouched. RX needs max frame + 1;
 * TX needs max frame. Successful Init clears callbacks, stats and sync hooks. */
int8_t MrLink_Init(MrLink_t *, const MrLink_Config_t *, uint8_t *, uint16_t,
                   uint8_t *, uint16_t);
uint16_t MrLink_MaxFrameSizeForConfig(const MrLink_Config_t *);
int8_t MrLink_SetSynchronization(MrLink_t *, const MrLink_Synchronization_t *);
int8_t MrLink_Reset(MrLink_t *);
/* PushBytes never calls business/error callbacks. Partial acceptance returns
 * BUF_FULL; exact accepted/dropped byte counts are in the stats snapshot. */
uint16_t MrLink_RxSpace(const MrLink_t *);
int8_t MrLink_PushBytes(MrLink_t *, const uint8_t *, uint16_t);
/* One task only. Recursive Parse/Dispatch/registration/reset returns BUSY.
 * Both Parse and Dispatch invoke handlers. Parse consumes one valid wire frame,
 * including a frame rejected by a length-checked subscription. Its payload is
 * valid until the next Parse/Dispatch/Reset. Callbacks may PushBytes or Build.
 * Dispatch processes at most the documented frame and consumed-byte budget.
 * Callback-supplied input does not extend a call beyond those budgets. */
int8_t MrLink_Dispatch(MrLink_t *);
int8_t MrLink_Parse(MrLink_t *, uint8_t *, const uint8_t **, uint16_t *);
uint16_t MrLink_Build(MrLink_t *, uint8_t, const uint8_t *, uint16_t);
/* Build buffer is task-owned scratch. Async transport MUST copy on submit or
 * otherwise hold a separate owned buffer until completion/abort. */
int8_t MrLink_RegisterHandler(MrLink_t *, uint8_t, MrLink_FrameHandler_t, void *);
int8_t MrLink_RegisterTypedHandler(MrLink_t *, uint8_t, uint16_t,
                                  MrLink_TypedHandler_t, void *);
int8_t MrLink_SetErrorHandler(MrLink_t *, MrLink_ErrorHandler_t, void *);
/* Legacy live view: use only in the consumer task, with reception stopped.
 * Use Snapshot for monitoring while an ISR feeds input. */
const MrLink_Stats_t *MrLink_GetStats(const MrLink_t *);
int8_t MrLink_GetStatsSnapshot(const MrLink_t *, MrLink_Stats_t *);
#ifdef __cplusplus
}
#endif

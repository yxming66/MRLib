#include "component/communication/mrlink/mrlink.h"
#include <stddef.h>
#include <string.h>
#include "component/crc16.h"

static uintptr_t Enter(const MrLink_t *p) {
  return p->sync.enter ? p->sync.enter(p->sync.ctx) : 0u;
}
static void Exit(const MrLink_t *p, uintptr_t token) {
  if (p->sync.exit) p->sync.exit(p->sync.ctx, token);
}
static MrLink_Config_t Config(const MrLink_Config_t *cfg) {
  MrLink_Config_t c = {MRLINK_MAX_PAYLOAD_DEFAULT, true,
                       MRLINK_DEFAULT_HEADER_0, MRLINK_DEFAULT_HEADER_1};
  if (cfg) c = *cfg;
  if (c.header_0 == 0u && c.header_1 == 0u) {
    c.header_0 = MRLINK_DEFAULT_HEADER_0; c.header_1 = MRLINK_DEFAULT_HEADER_1;
  }
  return c;
}
uint16_t MrLink_MaxFrameSizeForConfig(const MrLink_Config_t *cfg) {
  const MrLink_Config_t c = Config(cfg);
  if (c.max_payload_size == 0u || c.max_payload_size > MRLINK_MAX_PAYLOAD_DEFAULT)
    return 0u;
  return (uint16_t)(4u + c.max_payload_size + (c.use_crc16 ? 2u : 0u));
}
int8_t MrLink_Init(MrLink_t *p, const MrLink_Config_t *cfg, uint8_t *rx,
                   uint16_t rx_size, uint8_t *tx, uint16_t tx_size) {
  if (!p || !rx || !tx) return MRLINK_ERR_NULL;
  const uint16_t max = MrLink_MaxFrameSizeForConfig(cfg);
  if (!max || rx_size <= max || tx_size < max) return MRLINK_ERR_ARGS;
  memset(p, 0, sizeof(*p));
  p->cfg = Config(cfg); p->rx_buf = rx; p->rx_buf_size = rx_size;
  p->tx_buf = tx; p->tx_buf_size = tx_size;
  return MRLINK_OK;
}
int8_t MrLink_SetSynchronization(MrLink_t *p, const MrLink_Synchronization_t *sync) {
  if (!p) return MRLINK_ERR_NULL;
  if (p->processing) return MRLINK_ERR_BUSY;
  if (sync && ((sync->enter == NULL) != (sync->exit == NULL))) return MRLINK_ERR_ARGS;
  const MrLink_Synchronization_t empty = {0};
  p->sync = sync ? *sync : empty;
  return MRLINK_OK;
}
int8_t MrLink_Reset(MrLink_t *p) {
  if (!p) return MRLINK_ERR_NULL;
  if (p->processing) return MRLINK_ERR_BUSY;
  const uintptr_t token = Enter(p);
  p->rx_head = p->rx_tail = 0u;
  p->pending_rx_dropped = 0u;
  memset(&p->stats, 0, sizeof(p->stats));
  memset(p->handlers, 0, sizeof(p->handlers));
  Exit(p, token);
  return MRLINK_OK;
}
static uint16_t SizeUnlocked(const MrLink_t *p) {
  return p->rx_head >= p->rx_tail ? (uint16_t)(p->rx_head - p->rx_tail)
      : (uint16_t)(p->rx_buf_size - p->rx_tail + p->rx_head);
}
static uint16_t Peek(MrLink_t *p, uint8_t *dst, uint16_t count) {
  const uintptr_t token = Enter(p);
  uint16_t n = SizeUnlocked(p);
  if (n > count) n = count;
  for (uint16_t i = 0; i < n; ++i)
    dst[i] = p->rx_buf[((uint32_t)p->rx_tail + i) % p->rx_buf_size];
  Exit(p, token);
  return n;
}
static void Drop(MrLink_t *p, uint16_t n) {
  const uintptr_t token = Enter(p);
  p->rx_tail = (uint16_t)(((uint32_t)p->rx_tail + n) % p->rx_buf_size);
  Exit(p, token);
}
uint16_t MrLink_RxSpace(const MrLink_t *p) {
  if (!p || !p->rx_buf || p->rx_buf_size < 2u) return 0u;
  const uintptr_t token = Enter(p);
  const uint16_t n = (uint16_t)(p->rx_buf_size - 1u - SizeUnlocked(p));
  Exit(p, token); return n;
}
int8_t MrLink_PushBytes(MrLink_t *p, const uint8_t *data, uint16_t len) {
  if (!p || (!data && len)) return MRLINK_ERR_NULL;
  if (!p->rx_buf || p->rx_buf_size < 2u) return MRLINK_ERR_ARGS;
  const uintptr_t token = Enter(p);
  const uint16_t space = (uint16_t)(p->rx_buf_size - 1u - SizeUnlocked(p));
  const uint16_t n = len < space ? len : space;
  const uint16_t first = n < p->rx_buf_size - p->rx_head ? n
      : (uint16_t)(p->rx_buf_size - p->rx_head);
  if (first) memcpy(p->rx_buf + p->rx_head, data, first);
  if (n > first) memcpy(p->rx_buf, data + first, n - first);
  p->rx_head = (uint16_t)(((uint32_t)p->rx_head + n) % p->rx_buf_size);
  p->stats.feed_calls++; p->stats.feed_bytes_total += n;
  p->stats.feed_bytes_dropped += (uint16_t)(len - n);
  const uint32_t dropped = (uint16_t)(len - n);
  if (UINT32_MAX - p->pending_rx_dropped < dropped) p->pending_rx_dropped = UINT32_MAX;
  else p->pending_rx_dropped += dropped;
  Exit(p, token);
  return n == len ? MRLINK_OK : MRLINK_ERR_BUF_FULL;
}
static void Report(MrLink_t *p, const MrLink_ErrorInfo_t *info) {
  if (p->error_handler) p->error_handler(info, p->error_ctx);
}
static void PendingDrops(MrLink_t *p) {
  const uintptr_t token = Enter(p);
  const uint32_t n = p->pending_rx_dropped;
  p->pending_rx_dropped = 0u;
  Exit(p, token);
  if (n) {
    const MrLink_ErrorInfo_t info = {.code = MRLINK_ERROR_RX_DROPPED,
      .dropped_bytes = n > UINT16_MAX ? UINT16_MAX : (uint16_t)n};
    Report(p, &info);
  }
}
static int8_t Process(MrLink_t *p, bool one, uint8_t *cmd_out,
                       const uint8_t **payload_out, uint16_t *length_out) {
  if (p->processing) return MRLINK_ERR_BUSY;
  if (!p->rx_buf || p->rx_buf_size < 2u) return MRLINK_ERR_ARGS;
  p->processing = true;
  PendingDrops(p); /* Reentry protection includes error callbacks. */
  uint32_t frames = 0u, bytes = 0u;
  const uint32_t frame_budget = one ? 1u : MRLINK_DISPATCH_MAX_FRAMES;
  while (frames < frame_budget && bytes < MRLINK_DISPATCH_MAX_BYTES) {
    uint8_t buf[MRLINK_MAX_FRAME_SIZE];
    uint16_t n = Peek(p, buf, 4u);
    if (!n) break;
    MrLink_ErrorInfo_t info = {0};
    if (buf[0] != p->cfg.header_0 || (n >= 2u && buf[1] != p->cfg.header_1)) {
      Drop(p, 1u); bytes++;
      p->stats.frame_rx_header_skip++;
      info.code = MRLINK_ERROR_HEADER_SKIP; info.available = n;
      Report(p, &info); continue;
    }
    if (n < 3u) break;
    const uint16_t len = buf[2];
    if (len > p->cfg.max_payload_size) {
      Drop(p, 1u); bytes++;
      p->stats.frame_rx_oversize++;
      info.code = MRLINK_ERROR_OVERSIZE; info.payload_len = len;
      info.expected_size = p->cfg.max_payload_size;
      Report(p, &info); continue;
    }
    const uint16_t total = (uint16_t)(4u + len + (p->cfg.use_crc16 ? 2u : 0u));
    if (total > MRLINK_DISPATCH_MAX_BYTES - bytes) break;
    n = Peek(p, buf, total);
    if (n < total) {
      p->stats.frame_rx_truncated++;
      info.code = MRLINK_ERROR_TRUNCATED; info.payload_len = len;
      info.expected_size = total; info.available = n;
      Report(p, &info); break;
    }
    if (p->cfg.use_crc16) {
      const uint16_t received = (uint16_t)(buf[total-2u] | ((uint16_t)buf[total-1u] << 8u));
      const uint16_t calculated = CRC16_Calc(buf, total-2u, CRC16_INIT);
      if (received != calculated) {
        Drop(p, 1u); bytes++; p->stats.frame_rx_crc_err++;
        info.code = MRLINK_ERROR_CRC; info.crc_received = received;
        info.crc_calculated = calculated; Report(p, &info); continue;
      }
    }
    const uint8_t cmd = buf[3];
    if (len) memcpy(p->frame_buf, buf+4u, len);
    Drop(p, total); bytes += total; frames++; p->stats.frame_rx_ok++;
    if (one) { *cmd_out = cmd; *payload_out = p->frame_buf; *length_out = len; }
    MrLink_HandlerEntry_t *entry = NULL;
    for (uint8_t i = 0; i < MRLINK_MAX_HANDLERS; ++i)
      if (p->handlers[i].used && p->handlers[i].cmd == cmd) { entry = &p->handlers[i]; break; }
    info.cmd = cmd; info.payload_len = len;
    if (!entry) {
      p->stats.frame_rx_unknown_cmd++; info.code = MRLINK_ERROR_UNKNOWN_CMD;
      Report(p, &info);
    } else if (entry->typed && entry->expected_size != len) {
      p->stats.frame_rx_size_mismatch++; info.code = MRLINK_ERROR_SIZE_MISMATCH;
      info.expected_size = entry->expected_size; Report(p, &info);
    } else if (entry->typed) {
      (void)entry->handler.typed(cmd, p->frame_buf, len, entry->ctx);
    } else {
      (void)entry->handler.raw(cmd, p->frame_buf, len, entry->ctx);
    }
  }
  p->processing = false;
  return frames ? MRLINK_OK : MRLINK_ERR;
}
int8_t MrLink_Dispatch(MrLink_t *p) {
  return p ? Process(p, false, NULL, NULL, NULL) : MRLINK_ERR_NULL;
}
int8_t MrLink_Parse(MrLink_t *p, uint8_t *cmd, const uint8_t **payload, uint16_t *len) {
  if (!p || !cmd || !payload || !len) return MRLINK_ERR_NULL;
  *payload = NULL; *len = 0u;
  return Process(p, true, cmd, payload, len);
}
uint16_t MrLink_Build(MrLink_t *p, uint8_t cmd, const uint8_t *data, uint16_t len) {
  if (!p || !p->tx_buf || len > p->cfg.max_payload_size || (!data && len)) return 0u;
  const uint16_t total = (uint16_t)(4u + len + (p->cfg.use_crc16 ? 2u : 0u));
  if (p->tx_buf_size < total) return 0u;
  uint8_t *b = p->tx_buf;
  /* Copy before writing header, including when source aliases the TX buffer. */
  if (len) memmove(b+4u, data, len);
  b[0] = p->cfg.header_0; b[1] = p->cfg.header_1; b[2] = (uint8_t)len; b[3] = cmd;
  if (p->cfg.use_crc16) {
    const uint16_t crc = CRC16_Calc(b, total-2u, CRC16_INIT);
    b[total-2u] = (uint8_t)crc; b[total-1u] = (uint8_t)(crc >> 8u);
  }
  p->stats.frame_tx_ok++;
  return total;
}
static MrLink_HandlerEntry_t *Find(MrLink_t *p, uint8_t cmd, bool allocate) {
  MrLink_HandlerEntry_t *free_entry = NULL;
  for (uint8_t i = 0u; i < MRLINK_MAX_HANDLERS; ++i) {
    if (p->handlers[i].used && p->handlers[i].cmd == cmd) return &p->handlers[i];
    if (!p->handlers[i].used && !free_entry) free_entry = &p->handlers[i];
  }
  return allocate ? free_entry : NULL;
}
int8_t MrLink_RegisterHandler(MrLink_t *p, uint8_t cmd, MrLink_FrameHandler_t fn, void *ctx) {
  if (!p) return MRLINK_ERR_NULL;
  if (p->processing) return MRLINK_ERR_BUSY;
  MrLink_HandlerEntry_t *e = Find(p, cmd, fn != NULL);
  if (!e) return fn ? MRLINK_ERR_ARGS : MRLINK_OK;
  memset(e, 0, sizeof(*e));
  if (fn) { e->used = true; e->cmd = cmd; e->handler.raw = fn; e->ctx = ctx; }
  return MRLINK_OK;
}
int8_t MrLink_RegisterTypedHandler(MrLink_t *p, uint8_t cmd, uint16_t size,
                                  MrLink_TypedHandler_t fn, void *ctx) {
  if (!fn) return MrLink_RegisterHandler(p, cmd, NULL, NULL);
  if (!p) return MRLINK_ERR_NULL;
  if (p->processing) return MRLINK_ERR_BUSY;
  if (size > p->cfg.max_payload_size) return MRLINK_ERR_ARGS;
  MrLink_HandlerEntry_t *e = Find(p, cmd, true);
  if (!e) return MRLINK_ERR_ARGS;
  memset(e, 0, sizeof(*e));
  e->used = true; e->cmd = cmd; e->typed = true; e->expected_size = size;
  e->handler.typed = fn; e->ctx = ctx;
  return MRLINK_OK;
}
int8_t MrLink_SetErrorHandler(MrLink_t *p, MrLink_ErrorHandler_t fn, void *ctx) {
  if (!p) return MRLINK_ERR_NULL;
  if (p->processing) return MRLINK_ERR_BUSY;
  p->error_handler = fn; p->error_ctx = ctx;
  return MRLINK_OK;
}
const MrLink_Stats_t *MrLink_GetStats(const MrLink_t *p) { return p ? &p->stats : NULL; }
int8_t MrLink_GetStatsSnapshot(const MrLink_t *p, MrLink_Stats_t *out) {
  if (!p || !out) return MRLINK_ERR_NULL;
  const uintptr_t token = Enter(p); *out = p->stats; Exit(p, token);
  return MRLINK_OK;
}

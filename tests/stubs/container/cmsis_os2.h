#pragma once
#include <stdint.h>
struct MrTestQueue {
  uint32_t size;
  int reset_status;
  int put_status;
  int resets;
  int puts;
};
typedef MrTestQueue* osMessageQueueId_t;
enum { osOK = 0 };
static inline uint32_t osMessageQueueGetMsgSize(osMessageQueueId_t q) { return q->size; }
static inline int osMessageQueueReset(osMessageQueueId_t q) {
  ++q->resets;
  return q->reset_status;
}
static inline int osMessageQueuePut(osMessageQueueId_t q, const void*, uint8_t, uint32_t) {
  ++q->puts;
  return q->put_status;
}

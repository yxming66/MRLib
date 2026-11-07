#include <stdio.h>
#include <string.h>

#include "mrlink/mrlink.h"

static int g_failures;
static unsigned g_handler_calls;
static unsigned g_error_calls;
static MrLink_ErrorCode_t g_last_error;
static bool g_saw_drop;

static void Expect(bool condition, const char *message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    ++g_failures;
  }
}

static int8_t CountHandler(uint8_t cmd, const uint8_t *payload,
                           uint16_t payload_len, void *ctx) {
  (void)cmd;
  (void)payload;
  (void)payload_len;
  (void)ctx;
  ++g_handler_calls;
  return MRLINK_OK;
}

static void CountError(const MrLink_ErrorInfo_t *info, void *ctx) {
  (void)ctx;
  if (info != NULL) {
    ++g_error_calls;
    g_last_error = info->code;
    if (info->code == MRLINK_ERROR_RX_DROPPED) g_saw_drop = true;
  }
}

static void TestCapacityAndDeferredOverflow(void) {
  MrLink_t link;
  uint8_t rx_storage[71]; /* capacity is 70 because one ring slot is reserved */
  uint8_t tx_storage[MRLINK_MAX_FRAME_SIZE];
  uint8_t input[72] = {0};
  MrLink_Config_t config = {MRLINK_MAX_PAYLOAD_DEFAULT, true, 0U, 0U};

  Expect(MrLink_Init(&link, &config, rx_storage, sizeof(rx_storage),
                     tx_storage, sizeof(tx_storage)) == MRLINK_OK,
         "an RX storage of max-frame+1 should be accepted");
  Expect(MrLink_SetErrorHandler(&link, CountError, NULL) == MRLINK_OK,
         "error callback should register");
  g_error_calls = 0U;
  g_saw_drop = false;
  Expect(MrLink_PushBytes(&link, input, sizeof(input)) == MRLINK_ERR_BUF_FULL,
         "overflow input should be accepted with dropped-byte accounting");
  Expect(g_error_calls == 0U,
         "RX overflow must not invoke a business callback from PushBytes");
  (void)MrLink_Dispatch(&link);
  Expect(g_error_calls > 0U && g_saw_drop,
         "Dispatch should report deferred RX overflow");
}

static void TestParseConsumesExactlyOneFrame(void) {
  MrLink_t link;
  uint8_t rx_storage[128];
  uint8_t tx_storage[MRLINK_MAX_FRAME_SIZE];
  uint8_t first_frame[MRLINK_MAX_FRAME_SIZE];
  uint8_t second_frame[MRLINK_MAX_FRAME_SIZE];
  const uint8_t first_payload[] = {0x11U, 0x22U};
  const uint8_t second_payload[] = {0x33U};
  MrLink_Config_t config = {MRLINK_MAX_PAYLOAD_DEFAULT, true, 0U, 0U};

  Expect(MrLink_Init(&link, &config, rx_storage, sizeof(rx_storage),
                     tx_storage, sizeof(tx_storage)) == MRLINK_OK,
         "MRLink should initialize with host-owned buffers");
  Expect(MrLink_RegisterHandler(&link, 0x10U, CountHandler, NULL) ==
             MRLINK_OK &&
             MrLink_RegisterHandler(&link, 0x20U, CountHandler, NULL) ==
                 MRLINK_OK,
         "test handlers should register");

  uint16_t first_size = MrLink_Build(&link, 0x10U, first_payload,
                                     sizeof(first_payload));
  Expect(first_size > 0U, "first test frame should build");
  memcpy(first_frame, tx_storage, first_size);
  uint16_t second_size = MrLink_Build(&link, 0x20U, second_payload,
                                      sizeof(second_payload));
  Expect(second_size > 0U, "second test frame should build");
  memcpy(second_frame, tx_storage, second_size);

  uint8_t stream[2U * MRLINK_MAX_FRAME_SIZE];
  memcpy(stream, first_frame, first_size);
  memcpy(stream + first_size, second_frame, second_size);
  Expect(MrLink_PushBytes(&link, stream,
                          (uint16_t)(first_size + second_size)) == MRLINK_OK,
         "two concatenated frames should enter the RX stream");

  g_handler_calls = 0U;
  uint8_t cmd = 0U;
  const uint8_t *payload = NULL;
  uint16_t payload_len = 0U;
  Expect(MrLink_Parse(&link, &cmd, &payload, &payload_len) == MRLINK_OK,
         "Parse should return the first complete frame");
  Expect(cmd == 0x10U && payload_len == sizeof(first_payload) &&
             memcmp(payload, first_payload, payload_len) == 0,
         "first Parse result should preserve first-frame contents");
  Expect(g_handler_calls == 1U,
         "one-frame Parse should call only the first frame handler");

  Expect(MrLink_Parse(&link, &cmd, &payload, &payload_len) == MRLINK_OK,
         "a second Parse should return the buffered second frame");
  Expect(cmd == 0x20U && payload_len == sizeof(second_payload) &&
             memcmp(payload, second_payload, payload_len) == 0,
         "second Parse result should preserve second-frame contents");
  Expect(g_handler_calls == 2U,
         "second Parse should dispatch the second handler exactly once");
}

static void TestDispatchStillConsumesAllFrames(void) {
  MrLink_t link;
  uint8_t rx_storage[128];
  uint8_t tx_storage[MRLINK_MAX_FRAME_SIZE];
  uint8_t frame_a[MRLINK_MAX_FRAME_SIZE];
  uint8_t frame_b[MRLINK_MAX_FRAME_SIZE];
  uint8_t stream[2U * MRLINK_MAX_FRAME_SIZE];
  const uint8_t payload[40] = {0x5aU};
  MrLink_Config_t config = {MRLINK_MAX_PAYLOAD_DEFAULT, true, 0U, 0U};

  Expect(MrLink_Init(&link, &config, rx_storage, sizeof(rx_storage),
                     tx_storage, sizeof(tx_storage)) == MRLINK_OK,
         "Dispatch test instance should initialize");
  Expect(MrLink_RegisterHandler(&link, 0x31U, CountHandler, NULL) ==
             MRLINK_OK &&
             MrLink_RegisterHandler(&link, 0x32U, CountHandler, NULL) ==
                 MRLINK_OK,
         "Dispatch test handlers should register");
  const uint16_t size_a = MrLink_Build(&link, 0x31U, payload, sizeof(payload));
  memcpy(frame_a, tx_storage, size_a);
  const uint16_t size_b = MrLink_Build(&link, 0x32U, payload, sizeof(payload));
  memcpy(frame_b, tx_storage, size_b);
  memcpy(stream, frame_a, size_a);
  memcpy(stream + size_a, frame_b, size_b);
  g_handler_calls = 0U;
  Expect(MrLink_PushBytes(&link, stream,
                          (uint16_t)(size_a + size_b)) == MRLINK_OK,
         "Dispatch should receive both concatenated frames");
  Expect(MrLink_Dispatch(&link) == MRLINK_OK && g_handler_calls == 2U,
         "Dispatch should continue to process all buffered complete frames");
  Expect(MrLink_Dispatch(&link) == MRLINK_ERR,
         "Dispatch should report no frame after consuming the stream");
}


static unsigned g_lock_depth, g_lock_calls, g_reentrant_calls;
static uintptr_t TestEnter(void *ctx) {
  (void)ctx; ++g_lock_calls; return g_lock_depth++;
}
static void TestExit(void *ctx, uintptr_t token) {
  (void)ctx; Expect(g_lock_depth == token + 1u, "synchronization must be paired");
  g_lock_depth = (unsigned)token;
}
static int8_t ReentrantHandler(uint8_t cmd, const uint8_t *data, uint16_t len, void *ctx) {
  (void)cmd; (void)data; (void)len;
  MrLink_t *link = (MrLink_t *)ctx;
  Expect(g_lock_depth == 0u, "business callback must run outside synchronization");
  Expect(MrLink_Dispatch(link) == MRLINK_ERR_BUSY, "recursive Dispatch must be rejected");
  Expect(MrLink_Reset(link) == MRLINK_ERR_BUSY, "callback Reset must not corrupt parser");
  Expect(MrLink_RegisterHandler(link, 0x41, NULL, NULL) == MRLINK_ERR_BUSY,
         "callback registration must not mutate active slots");
  ++g_reentrant_calls;
  /* Feed forever if dispatch has no work budget. */
  const uint16_t n = MrLink_Build(link, 0x41, NULL, 0u);
  Expect(MrLink_PushBytes(link, link->tx_buf, n) == MRLINK_OK, "callback may enqueue more bytes");
  return MRLINK_OK;
}
static void TestBudgetsAndSynchronization(void) {
  MrLink_t link;
  uint8_t rx[256], tx[70];
  Expect(MrLink_Init(&link, NULL, rx, sizeof(rx), tx, sizeof(tx)) == MRLINK_OK, "init budget test");
  MrLink_Synchronization_t sync = {TestEnter, TestExit, NULL};
  Expect(MrLink_SetSynchronization(&link, &sync) == MRLINK_OK, "paired hooks accepted");
  MrLink_Synchronization_t bad = {TestEnter, NULL, NULL};
  Expect(MrLink_SetSynchronization(&link, &bad) == MRLINK_ERR_ARGS, "unpaired hooks rejected");
  Expect(MrLink_RegisterHandler(&link, 0x41, ReentrantHandler, &link) == MRLINK_OK, "register reentry test");
  const uint16_t n = MrLink_Build(&link, 0x41, NULL, 0u);
  Expect(MrLink_PushBytes(&link, tx, n) == MRLINK_OK, "feed budget seed");
  g_reentrant_calls = 0;
  Expect(MrLink_Dispatch(&link) == MRLINK_OK, "bounded dispatch returns");
  Expect(g_reentrant_calls == MRLINK_DISPATCH_MAX_FRAMES, "exact frame budget despite callback feeding");
  MrLink_Stats_t stats;
  Expect(MrLink_GetStatsSnapshot(&link, &stats) == MRLINK_OK &&
         stats.frame_rx_ok == g_reentrant_calls, "stats snapshot counts wire frames");
  Expect(g_lock_calls > 0 && g_lock_depth == 0u, "hooks cover ring and stats without leakage");
}
static void TestNoiseSplitCrcAndAliasing(void) {
  MrLink_t link;
  uint8_t rx[512], tx[70], frame[70], payload[64];
  for (uint8_t i = 0u; i < sizeof(payload); ++i) payload[i] = i;
  Expect(MrLink_Init(&link, NULL, rx, 70, tx, sizeof(tx)) == MRLINK_ERR_ARGS,
         "max-frame-sized RX storage is one byte too small");
  Expect(MrLink_Init(&link, NULL, rx, sizeof(rx), tx, sizeof(tx)) == MRLINK_OK, "init scan test");
  Expect(MrLink_RegisterHandler(&link, 0x10, CountHandler, NULL) == MRLINK_OK, "register scan test");
  const uint16_t n = MrLink_Build(&link, 0x10, payload, sizeof(payload));
  memcpy(frame, tx, n);
  uint8_t noise[80] = {0};
  MrLink_PushBytes(&link, noise, sizeof(noise));
  MrLink_PushBytes(&link, frame, n);
  g_handler_calls = 0;
  Expect(MrLink_Dispatch(&link) == MRLINK_OK && g_handler_calls == 1,
         "noise beyond old peek boundary must not stall a buffered max frame");
  for (uint16_t split = 1; split < n; ++split) {
    MrLink_PushBytes(&link, frame, split);
    Expect(MrLink_Dispatch(&link) == MRLINK_ERR, "partial input must wait");
    MrLink_PushBytes(&link, frame+split, (uint16_t)(n-split));
    Expect(MrLink_Dispatch(&link) == MRLINK_OK, "every split point must recover one frame");
  }
  frame[n-1u] ^= 1u;
  MrLink_PushBytes(&link, frame, n);
  Expect(MrLink_Dispatch(&link) == MRLINK_ERR, "bad CRC not accepted");
  frame[n-1u] ^= 1u;
  MrLink_PushBytes(&link, frame, n);
  Expect(MrLink_Dispatch(&link) == MRLINK_OK, "valid frame after corrupt one recovers");
  memcpy(tx, payload, sizeof(payload));
  Expect(MrLink_Build(&link, 0x10, tx, sizeof(payload)) == n && memcmp(tx, frame, n) == 0,
         "Build permits payload aliasing TX buffer");
  /* Independently calculated CRC16/IBM-SDLC-without-xorout golden frame. */
  const uint8_t golden[] = {0x4d,0x52,0x02,0x10,0x11,0x22,0x2e,0x6e};
  const uint16_t gn = MrLink_Build(&link, 0x10, golden+4u, 2u);
  Expect(gn == sizeof(golden) && memcmp(tx, golden, sizeof(golden)) == 0, "wire header and length stable");
  /* A typed mismatch is still a consumed valid wire frame in pull mode. */
  MrLink_Reset(&link);
  MrLink_PushBytes(&link, frame, n);
  uint8_t cmd; const uint8_t *out; uint16_t len;
  Expect(MrLink_Parse(&link, &cmd, &out, &len) == MRLINK_OK && len == 64u &&
         memcmp(out, payload, len) == 0, "maximum payload survives reset and pull");
}

int main(void) {
  TestNoiseSplitCrcAndAliasing();
  TestBudgetsAndSynchronization();
  TestCapacityAndDeferredOverflow();
  TestParseConsumesExactlyOneFrame();
  TestDispatchStillConsumesAllFrames();
  if (g_failures != 0) {
    printf("%d MRLink host test(s) failed\n", g_failures);
    return 1;
  }
  printf("MRLink host tests passed\n");
  return 0;
}

// Real FDCAN manager + board filters, deterministic HAL/RTOS; no real bus.
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/fdcan.h"
#include "fdcan.h"

uint32_t fdcan_primask;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static FDCAN_GlobalTypeDef registers[3];
static unsigned starts[3], stops[3], tx_count[3], free_tx[3], notifications[3];
static FDCAN_HandleTypeDef *fail_start, *fail_stop, *fail_notify;
static bool fail_mutex, fail_delete, fail_abort, fail_add;
static osKernelState_t kernel;
static unsigned mutex, allocations, queues, observers, filters, globals;
static FDCAN_TxHeaderTypeDef last_tx;
static uint8_t last_data[64];
typedef struct { FDCAN_RxHeaderTypeDef header; uint8_t data[64]; } Rx;
static struct { Rx frames[32]; unsigned head, tail; } rx[3][2];
typedef struct { unsigned head, count, capacity; BSP_FDCAN_Message_t data[32]; } Queue;

static unsigned Index(FDCAN_HandleTypeDef* h) {
  assert(h == &hfdcan1 || h == &hfdcan2 || h == &hfdcan3);
  return h == &hfdcan1 ? 0 : h == &hfdcan2 ? 1 : 2;
}
void* BSP_Malloc(size_t size) { ++allocations; return calloc(1, size); }
void BSP_Free(void* p) { assert(allocations); --allocations; free(p); }
osKernelState_t osKernelGetState(void) { return kernel; }
osMutexId_t osMutexNew(const void* attr) { (void)attr; return fail_mutex ? NULL : &mutex; }
osStatus_t osMutexAcquire(osMutexId_t id, uint32_t timeout) {
  (void)timeout; assert(id == &mutex && !mutex); mutex = 1; return osOK;
}
osStatus_t osMutexRelease(osMutexId_t id) { assert(id == &mutex && mutex); mutex = 0; return osOK; }
osMessageQueueId_t osMessageQueueNew(uint32_t n, uint32_t size, const void* attr) {
  (void)attr; assert(size == sizeof(BSP_FDCAN_Message_t) && n <= 32);
  Queue* q = calloc(1, sizeof(*q)); assert(q); q->capacity = n; ++queues; return q;
}
osStatus_t osMessageQueueDelete(osMessageQueueId_t id) {
  if (fail_delete) return osError;
  assert(queues); --queues; free(id); return osOK;
}
osStatus_t osMessageQueuePut(osMessageQueueId_t id, const void* data, uint8_t priority, uint32_t timeout) {
  (void)priority; assert(timeout == 0); Queue* q = id;
  if (q->count == q->capacity) return osError;
  q->data[(q->head + q->count++) % q->capacity] = *(const BSP_FDCAN_Message_t*)data;
  return osOK;
}
osStatus_t osMessageQueueGet(osMessageQueueId_t id, void* data, uint8_t* priority, uint32_t timeout) {
  (void)priority; (void)timeout; Queue* q = id;
  if (!q->count) return osError;
  *(BSP_FDCAN_Message_t*)data = q->data[q->head]; q->head = (q->head + 1) % q->capacity; --q->count;
  return osOK;
}
uint32_t osMessageQueueGetCount(osMessageQueueId_t id) { return ((Queue*)id)->count; }
uint64_t BSP_TIME_Get_us(void) { return 123456; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* h) {
  ++starts[Index(h)];
  if (h == fail_start) return HAL_ERROR;
  assert(h->State == HAL_FDCAN_STATE_READY); h->State = HAL_FDCAN_STATE_BUSY; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef* h) {
  ++stops[Index(h)];
  if (h == fail_stop) return HAL_ERROR;
  h->State = HAL_FDCAN_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* h, uint32_t mask, uint32_t buffers) {
  assert(!buffers); if (h == fail_notify) return HAL_ERROR;
  notifications[Index(h)] |= mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef* h, uint32_t mask) {
  notifications[Index(h)] &= ~mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef* h, FDCAN_ProtocolStatusTypeDef* s) {
  s->BusOff = !!(h->Instance->PSR & FDCAN_PSR_BO); return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef* h, FDCAN_ErrorCountersTypeDef* s) {
  (void)h; s->TxErrorCnt = s->RxErrorCnt = 0; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef* h, uint32_t mask) {
  if (fail_abort) return HAL_ERROR;
  h->Instance->TXBRP &= ~mask; return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef* h) { return free_tx[Index(h)]; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* h, const FDCAN_TxHeaderTypeDef* header, const uint8_t* data) {
  if (fail_add) return HAL_ERROR;
  unsigned i = Index(h); assert(h->State == HAL_FDCAN_STATE_BUSY && free_tx[i]);
  ++tx_count[i]; --free_tx[i]; last_tx = *header;
  const unsigned lengths[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  memcpy(last_data, data, lengths[header->DataLength]); return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef* h, uint32_t fifo) {
  assert(fifo < 2); const unsigned i = Index(h); return rx[i][fifo].tail - rx[i][fifo].head;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* h, uint32_t fifo, FDCAN_RxHeaderTypeDef* header, uint8_t* data) {
  const unsigned i = Index(h); assert(rx[i][fifo].head < rx[i][fifo].tail);
  const Rx* frame = &rx[i][fifo].frames[rx[i][fifo].head++];
  *header = frame->header; memcpy(data, frame->data, sizeof(frame->data)); return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* h, const FDCAN_FilterTypeDef* filter) {
  (void)Index(h); assert(filter->FilterIndex == 0 && filter->FilterID1 == 0 && filter->FilterID2 == 0);
  assert(filter->IdType == FDCAN_STANDARD_ID || filter->IdType == FDCAN_EXTENDED_ID);
  ++filters; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* h, uint32_t standard, uint32_t extended, uint32_t rs, uint32_t re) {
  const unsigned i = Index(h); assert(standard == extended && rs == FDCAN_FILTER_REMOTE && re == FDCAN_FILTER_REMOTE);
  assert(standard == (i ? FDCAN_ACCEPT_IN_RX_FIFO1 : FDCAN_ACCEPT_IN_RX_FIFO0)); ++globals; return HAL_OK;
}
static void Setup(void) {
  FDCAN_HandleTypeDef* handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
  for (unsigned i = 0; i < 3; ++i) {
    *handles[i] = (FDCAN_HandleTypeDef){0};
    handles[i]->Instance = &registers[i];
    handles[i]->Init = (FDCAN_InitTypeDef){.FrameFormat = FDCAN_FRAME_CLASSIC,
        .TxFifoQueueElmtsNbr = 32, .TxElmtSize = FDCAN_DATA_BYTES_8,
        .RxFifo0ElmtsNbr = 16, .RxFifo1ElmtsNbr = 16,
        .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8, .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
        .StdFiltersNbr = 1, .ExtFiltersNbr = 1};
    handles[i]->msgRam.StandardFilterSA = SRAMCAN_BASE + 0x400u * i;
    handles[i]->msgRam.EndAddress = SRAMCAN_BASE + 0x400u * i + 0x300u;
  }
}
static void Observer(void) { ++observers; }
static void OtherObserver(void) { assert(0); }
static void AddRx(unsigned index, unsigned fifo, unsigned id, uint8_t value) {
  assert(rx[index][fifo].tail < 32);
  Rx* f = &rx[index][fifo].frames[rx[index][fifo].tail++];
  f->header = (FDCAN_RxHeaderTypeDef){id, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, 8};
  memset(f->data, value, sizeof(f->data));
}
static void TestConfiguration(void) {
  assert(BSP_FDCAN_Init() != BSP_OK);
  BSP_FDCAN_Endpoint_t table[] = {{"second", &hfdcan2, BSP_FDCAN_RX_FIFO1}, {"first", &hfdcan1, BSP_FDCAN_RX_FIFO0}};
  table[1].name = "second"; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  table[1].name = "first"; table[1].handle = &hfdcan2; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  FDCAN_HandleTypeDef alias = hfdcan2; table[1].handle = &alias;
  assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  table[1].handle = &hfdcan1; table[0].rx_fifos = 4; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  table[0].rx_fifos = BSP_FDCAN_RX_FIFO1;
  const uint32_t end = hfdcan1.msgRam.EndAddress;
  hfdcan1.msgRam.EndAddress = hfdcan2.msgRam.EndAddress; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  hfdcan1.msgRam.EndAddress = end;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK); hfdcan2.Init.RxFifo1ElmtsNbr = 16;
  hfdcan2.State = HAL_FDCAN_STATE_BUSY; assert(BSP_FDCAN_Configure(table, 2) != BSP_OK); hfdcan2.State = HAL_FDCAN_STATE_READY;
  assert(BSP_FDCAN_GetHandle(BSP_FDCAN_1) == NULL); // None of the invalid tables was published.
  assert(BSP_FDCAN_Configure(NULL, 1) != BSP_OK && BSP_FDCAN_Configure(table, 4) != BSP_OK);
  assert(BSP_FDCAN_Configure(table, 2) == BSP_OK);
  assert(BSP_FDCAN_Configure(table, 2) != BSP_OK);
  assert(BSP_FDCAN_FindEndpoint("second") == BSP_FDCAN_1 && BSP_FDCAN_GetHandle(BSP_FDCAN_1) == &hfdcan2);
  assert(BSP_FDCAN_FindEndpoint(NULL) == BSP_FDCAN_ERR && BSP_FDCAN_FindEndpoint("absent") == BSP_FDCAN_ERR);
  assert(BSP_FDCAN_GetHandle((BSP_FDCAN_t)-1) == NULL && BSP_FDCAN_GetHandle(BSP_FDCAN_3) == NULL);
  assert(!starts[0] && !starts[1] && !allocations && !filters); // Registration is inert.
}
static void TestInit(void) {
  assert(BSP_FDCAN_Init() != BSP_OK); kernel = osKernelRunning;
  fail_mutex = true; assert(BSP_FDCAN_Init() != BSP_OK); fail_mutex = false;
  fail_notify = &hfdcan1;
  assert(BSP_FDCAN_Init() != BSP_OK && hfdcan2.State == HAL_FDCAN_STATE_READY);
  assert(!notifications[0] && !notifications[1]); fail_notify = NULL;
  fail_start = &hfdcan1;
  assert(BSP_FDCAN_Init() != BSP_OK && hfdcan2.State == HAL_FDCAN_STATE_READY);
  assert(BSP_FDCAN_RegisterLatestId(BSP_FDCAN_1, 0x201) != BSP_OK); fail_start = NULL;
  assert(BSP_FDCAN_Init() == BSP_OK && !starts[2]);
  assert(BSP_FDCAN_Init() != BSP_OK);
  assert(notifications[1] & FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  assert(!(notifications[1] & FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}
static void TestReceive(void) {
  assert(BSP_FDCAN_RegisterLatestId(BSP_FDCAN_1, 0x201) == BSP_OK);
  assert(BSP_FDCAN_RegisterLatestId(BSP_FDCAN_2, 0x201) == BSP_OK);
  assert(BSP_FDCAN_RegisterLatestId(BSP_FDCAN_3, 0x201) != BSP_OK);
  assert(BSP_FDCAN_RegisterLatestId((BSP_FDCAN_t)-1, 0x201) != BSP_OK);
  assert(BSP_FDCAN_RegisterCallback(BSP_FDCAN_1, HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB, Observer) == BSP_OK);
  assert(BSP_FDCAN_RegisterCallback(BSP_FDCAN_1, HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB, OtherObserver) != BSP_OK);
  AddRx(1, 1, 0x201, 22); AddRx(0, 0, 0x201, 11);
  FDCAN_HandleTypeDef alias = hfdcan2;
  HAL_FDCAN_RxFifo1Callback(&alias, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  assert(!observers && rx[1][1].head == 0);
  HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(rx[0][0].head == 0 && rx[1][1].head == 0);
  HAL_FDCAN_RxFifo1Callback(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  assert(observers == 1 && rx[0][0].head == 0); // Never scans/drains another controller.
  BSP_FDCAN_Message_t msg;
  assert(BSP_FDCAN_GetLatestMessage(BSP_FDCAN_1, 0x201, &msg) == BSP_OK && msg.data[0] == 22 && msg.timestamp == 123);
  assert(BSP_FDCAN_GetLatestMessage(BSP_FDCAN_2, 0x201, &msg) != BSP_OK);
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(BSP_FDCAN_GetLatestMessage(BSP_FDCAN_2, 0x201, &msg) == BSP_OK && msg.data[0] == 11);
  assert(BSP_FDCAN_RegisterId(BSP_FDCAN_1, 0x101, 4) == BSP_OK);
  AddRx(1, 1, 0x101, 33); HAL_FDCAN_RxFifo1Callback(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  assert(BSP_FDCAN_GetQueueCount(BSP_FDCAN_1, 0x101) == 1);
  assert(BSP_FDCAN_GetMessage(BSP_FDCAN_1, 0x101, &msg, 0) == BSP_OK && msg.data[0] == 33);
  for (unsigned i = 0; i < 17; ++i) AddRx(1, 1, 0x201, (uint8_t)i);
  HAL_FDCAN_RxFifo1Callback(&hfdcan2, FDCAN_IT_RX_FIFO1_FULL);
  assert(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, 1) == 0);
  HAL_FDCAN_RxFifo1Callback(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  assert(BSP_FDCAN_FlushQueue(BSP_FDCAN_1, 0x201) == BSP_OK);
  assert(BSP_FDCAN_GetLatestMessage(BSP_FDCAN_1, 0x201, &msg) != BSP_OK);
  assert(BSP_FDCAN_UnregisterCallback(BSP_FDCAN_1, HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB, OtherObserver) != BSP_OK);
  assert(BSP_FDCAN_UnregisterCallback(BSP_FDCAN_1, HAL_FDCAN_RX_FIFO1_MSG_PENDING_CB, Observer) == BSP_OK);
  fail_delete = true; assert(BSP_FDCAN_UnregisterId(BSP_FDCAN_1, 0x101) != BSP_OK);
  fail_delete = false; assert(BSP_FDCAN_UnregisterId(BSP_FDCAN_1, 0x101) == BSP_OK);
  assert(BSP_FDCAN_UnregisterId(BSP_FDCAN_1, 0x201) == BSP_OK);
  assert(BSP_FDCAN_UnregisterId(BSP_FDCAN_2, 0x201) == BSP_OK && !allocations && !queues);
}
static void TestTransmit(void) {
  uint8_t data[9] = {1,2,3,4,5,6,7,8,9};
  free_tx[1] = 1;
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 8) == BSP_OK && tx_count[1] == 1);
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x800, data, 8) != BSP_OK);
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 9) != BSP_OK);
  assert(BSP_FDCAN_Transmit((BSP_FDCAN_t)-1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 8) != BSP_OK);
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 8) == BSP_OK);
  assert(notifications[1] & FDCAN_IT_TX_FIFO_EMPTY);
  free_tx[1] = 1; HAL_FDCAN_TxFifoEmptyCallback(&hfdcan2);
  assert(tx_count[1] == 2 && !(notifications[1] & FDCAN_IT_TX_FIFO_EMPTY));
  const unsigned first_starts = starts[0], second_starts = starts[1];
  fail_stop = &hfdcan2;
  assert(BSP_FDCAN_Recover(BSP_FDCAN_1) != BSP_OK && starts[1] == second_starts);
  free_tx[1] = 1;
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 8) != BSP_OK && tx_count[1] == 2);
  fail_stop = NULL;
  hfdcan2.Instance->TXBRP = 1; fail_abort = true;
  assert(BSP_FDCAN_Recover(BSP_FDCAN_1) != BSP_OK && starts[1] == second_starts);
  fail_abort = false;
  assert(BSP_FDCAN_Recover(BSP_FDCAN_1) == BSP_OK && starts[0] == first_starts && starts[1] == second_starts + 1);
  assert(!filters && !globals); // Recovery never rebuilds filters or shared RAM.
}
static void TestFdFrames(void) {
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  const BSP_FDCAN_Endpoint_t table[] = {{"fd", &hfdcan2, BSP_FDCAN_RX_FIFO1}};
  assert(BSP_FDCAN_Configure(table, 1) != BSP_OK); // FD cannot use 8-byte RAM elements.
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  assert(BSP_FDCAN_Configure(table, 1) != BSP_OK);
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  assert(BSP_FDCAN_Configure(table, 1) == BSP_OK);
  kernel = osKernelRunning;
  assert(BSP_FDCAN_Init() == BSP_OK);
  uint8_t data[9] = {1,2,3,4,5,6,7,8,9};
  free_tx[1] = 2;
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_DATA, 0x100, data, 9) == BSP_OK);
  assert(last_tx.FDFormat == FDCAN_FD_CAN && last_tx.BitRateSwitch == FDCAN_BRS_ON && last_tx.DataLength == FDCAN_DLC_BYTES_12);
  assert(last_data[8] == 9 && last_data[9] == 0 && last_data[10] == 0 && last_data[11] == 0);
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_REMOTE, 0x100, NULL, 9) != BSP_OK);
  assert(BSP_FDCAN_Transmit(BSP_FDCAN_1, BSP_FDCAN_FORMAT_STD_REMOTE, 0x100, NULL, 8) == BSP_OK);
  assert(last_tx.FDFormat == FDCAN_CLASSIC_CAN && last_tx.BitRateSwitch == FDCAN_BRS_OFF);
}
extern int Board_RegisterFdcanEndpoints(void);
int main(int argc, char** argv) {
  Setup();
  if (argc > 1 && !strcmp(argv[1], "fd")) { TestFdFrames(); return 0; }
  if (argc > 1 && !strcmp(argv[1], "board")) {
    assert(Board_RegisterFdcanEndpoints() && filters == 6 && globals == 3);
    assert(!starts[0] && !starts[1] && !starts[2]);
    return 0;
  }
  TestConfiguration();
  if (argc > 1 && !strcmp(argv[1], "locked")) {
    kernel = osKernelRunning; fail_start = &hfdcan1; fail_stop = &hfdcan2;
    assert(BSP_FDCAN_Init() != BSP_OK); const unsigned count = starts[1];
    fail_start = fail_stop = NULL;
    assert(BSP_FDCAN_Init() != BSP_OK && starts[1] == count);
    return 0;
  }
  TestInit(); TestReceive(); TestTransmit();
  assert(!fdcan_primask);
  return 0;
}

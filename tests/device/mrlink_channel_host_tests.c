#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bsp/dma.h"
#include "bsp/gpio.h"
#include "bsp/spi.h"
#include "cmsis_os2.h"
#include "device/bmi088.h"
#include "device/dr16.h"
#include "mrlink/mrlink_channel.h"
uint32_t io_primask;
USART_TypeDef io_uart_instances[6];
static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart1 = {USART1, &dma, 0, 0, 0, 0}, huart5 = {UART5, &dma, 0, 0, 0, 0},
                   huart7 = {UART7, &dma, 0, 0, 0, 0}, huart8 = {UART8, &dma, 0, 0, 0, 0},
                   huart9 = {UART9, &dma, 0, 0, 0, 0}, huart10 = {USART10, &dma, 0, 0, 0, 0};
SPI_HandleTypeDef hspi2 = {SPI2, &dma, 0};
static uint8_t *hal_tx, *hal_rx;
static unsigned rx_starts, tx_starts, dma_completed, done_calls, error_calls, sync_calls;
static bool abort_fail, reject_cache;
static HAL_StatusTypeDef spi_result = HAL_OK;
static bool gpio_pins[BSP_GPIO_NUM];
static uint32_t thread_flags;
osThreadId_t osThreadGetId(void) { return (void*)1; }
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags) {
  (void)id;
  thread_flags |= flags;
  return thread_flags;
}
uint32_t osThreadFlagsClear(uint32_t flags) {
  uint32_t old = thread_flags;
  thread_flags &= ~flags;
  return old;
}
uint32_t osThreadFlagsWait(uint32_t mask, uint32_t options, uint32_t timeout) {
  (void)timeout;
  uint32_t result = thread_flags & mask;
  if (!result || (options == osFlagsWaitAll && result != mask)) return osFlagsErrorTimeout;
  thread_flags &= ~result;
  return result;
}
int8_t BSP_GPIO_RegisterCallback(BSP_GPIO_t p, void (*fn)(void)) {
  (void)p;
  (void)fn;
  return 0;
}
int8_t BSP_GPIO_EnableIRQ(BSP_GPIO_t p) {
  (void)p;
  return 0;
}
int8_t BSP_GPIO_DisableIRQ(BSP_GPIO_t p) {
  (void)p;
  return 0;
}
int8_t BSP_GPIO_WritePin(BSP_GPIO_t p, bool value) {
  gpio_pins[p] = value;
  return 0;
}
bool BSP_GPIO_ReadPin(BSP_GPIO_t p) { return gpio_pins[p]; }
uint64_t BSP_TIME_Get_us(void) { return 1000; }
int8_t BSP_TIME_Delay(uint32_t ms) {
  (void)ms;
  return 0;
}

static char spi_trace[16];
static unsigned spi_count;
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  (void)n;
  if (h->gState != HAL_UART_STATE_READY) return HAL_BUSY;
  hal_tx = p;
  h->gState = 1;
  ++tx_starts;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  return HAL_UART_Transmit_DMA(h, p, n);
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  (void)n;
  if (h->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
  hal_rx = p;
  h->RxState = 1;
  ++rx_starts;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  return HAL_UARTEx_ReceiveToIdle_DMA(h, p, n);
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  return HAL_UARTEx_ReceiveToIdle_DMA(h, p, n);
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  return HAL_UARTEx_ReceiveToIdle_DMA(h, p, n);
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* h) {
  if (abort_fail) return HAL_ERROR;
  h->RxState = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef* h) {
  if (abort_fail) return HAL_ERROR;
  h->gState = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* h, uint8_t* p, uint16_t n, uint32_t timeout) {
  (void)h;
  (void)p;
  (void)n;
  (void)timeout;
  spi_trace[spi_count++ % sizeof(spi_trace)] = 'T';
  return spi_result;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* h, uint8_t* p, uint16_t n, uint32_t timeout) {
  (void)h;
  (void)timeout;
  memset(p, 0x5a, n);
  if (n == 2) {
    p[0] = 0x0f;
    p[1] = 0x1e;
  }
  spi_trace[spi_count++ % sizeof(spi_trace)] = 'R';
  return spi_result;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* h, uint8_t* p, uint8_t* r, uint16_t n,
                                          uint32_t t) {
  (void)p;
  return HAL_SPI_Receive(h, r, n, t);
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  (void)h;
  (void)p;
  (void)n;
  return spi_result;
}
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef* h, uint8_t* p, uint16_t n) {
  (void)h;
  (void)n;
  hal_rx = p;
  return spi_result;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef* h, uint8_t* p, uint8_t* r,
                                              uint16_t n) {
  (void)p;
  return HAL_SPI_Receive_DMA(h, r, n);
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef* h) {
  (void)h;
  return spi_result;
}
static bool fd_registered, fd_has_message, fd_unregister_fail;
static BSP_FDCAN_Message_t fd_message;
int8_t BSP_FDCAN_RegisterId(BSP_FDCAN_t b, uint32_t id, uint8_t n) {
  (void)b;
  (void)id;
  (void)n;
  if (fd_registered) return BSP_ERR;
  fd_registered = true;
  return BSP_OK;
}
int8_t BSP_FDCAN_UnregisterId(BSP_FDCAN_t b, uint32_t id) {
  (void)b;
  (void)id;
  if (fd_unregister_fail) return BSP_ERR;
  fd_registered = false;
  return BSP_OK;
}
int8_t BSP_FDCAN_GetMessage(BSP_FDCAN_t b, uint32_t id, BSP_FDCAN_Message_t* out, uint32_t t) {
  (void)b;
  (void)id;
  (void)t;
  if (!fd_has_message) return BSP_ERR;
  *out = fd_message;
  fd_has_message = false;
  return BSP_OK;
}
int8_t BSP_FDCAN_Transmit(BSP_FDCAN_t b, BSP_FDCAN_Format_t f, uint32_t id, const uint8_t* p,
                          uint8_t n) {
  (void)b;
  (void)f;
  (void)id;
  memcpy(fd_message.data, p, n);
  return BSP_OK;
}
static bool Cache(BSP_DMA_Operation_t op, void* p, uint16_t n, void* ctx) {
  (void)p;
  (void)n;
  (void)ctx;
  ++sync_calls;
  if (reject_cache) return false;
  if (op == BSP_DMA_COMPLETE_RX) ++dma_completed;
  return true;
}
static void Done(void* ctx) {
  (void)ctx;
  ++done_calls;
}
static void Error(void* ctx) {
  (void)ctx;
  ++error_calls;
}
static void Legacy(void) {}
static void SpiDone(void) {
  assert(dma_completed);
  ++done_calls;
}
static void SpiError(void) { ++error_calls; }
static void TestUart(void) {
  for (unsigned id = 0; id < BSP_UART_NUM; ++id) {
    MrLink_Channel_t channel = {0}, other = {0};
    uint8_t slots[3][32] = {{0}};
    volatile uint16_t lens[3] = {0};
    MrLink_ChannelUartConfig_t cfg = {(BSP_UART_t)id, &slots[0][0], 32, 3, lens, NULL, NULL};
    assert(BSP_UART_RegisterCallback((BSP_UART_t)id, BSP_UART_TX_CPLT_CB, Legacy) == BSP_OK);
    assert(MrLink_Channel_InitUart(&channel, &cfg) != MRLINK_CHANNEL_OK);
    assert(channel.hardware == MRLINK_CHANNEL_HW_NONE);
    assert(BSP_UART_RegisterCallback((BSP_UART_t)id, BSP_UART_TX_CPLT_CB, NULL) == BSP_OK);
    assert(MrLink_Channel_InitUart(&channel, &cfg) == MRLINK_CHANNEL_OK);
    assert(MrLink_Channel_InitUart(&channel, &cfg) == MRLINK_CHANNEL_ERR_BUSY);
    assert(MrLink_Channel_InitUart(&other, &cfg) != MRLINK_CHANNEL_OK);
    assert(BSP_UART_RegisterCallback((BSP_UART_t)id, BSP_UART_TX_CPLT_CB, Legacy) != BSP_OK);
    assert(MrLink_Channel_RegisterTxCallbacks(&channel, Done, Error, NULL) == MRLINK_CHANNEL_OK);
    UART_HandleTypeDef* h = BSP_UART_GetHandle((BSP_UART_t)id);
    assert(MrLink_Channel_StartRx(&channel) == MRLINK_CHANNEL_OK);
    unsigned starts = rx_starts;
    assert(MrLink_Channel_StartRx(&channel) == MRLINK_CHANNEL_OK && rx_starts == starts);
    for (unsigned i = 0; i < 10; ++i) hal_rx[i] = (uint8_t)i;
    h->RxState = 0;
    HAL_UARTEx_RxEventCallback(h, 10);
    uint8_t scratch[4];
    assert(MrLink_Channel_PopRx(&channel, scratch, 4) == 4 && scratch[3] == 3);
    assert(MrLink_Channel_PopRx(&channel, scratch, 4) == 4 && scratch[0] == 4);
    assert(MrLink_Channel_PopRx(&channel, scratch, 4) == 2 && scratch[1] == 9);
    assert(MrLink_Channel_PopRx(&channel, scratch, 4) == 0);
    starts = rx_starts;
    h->RxState = 1;
    h->ErrorCode = 0;
    HAL_UART_ErrorCallback(h);
    (void)MrLink_Channel_PopRx(&channel, scratch, 4);
    assert(rx_starts > starts);  // task polling recovers an RX HAL that was still busy in ISR
    uint8_t source[] = {1, 2, 3};
    assert(MrLink_Channel_Send(&channel, source, 3) == MRLINK_CHANNEL_OK);
    source[0] = 9;
    assert(hal_tx[0] == 1);
    assert(MrLink_Channel_Send(&channel, source, 3) == MRLINK_CHANNEL_ERR_BUSY && hal_tx[0] == 1);
    h->ErrorCode = 0;
    h->RxState = 0;
    HAL_UART_ErrorCallback(h);
    assert(MrLink_Channel_Send(&channel, source, 3) == MRLINK_CHANNEL_ERR_BUSY);
    h->gState = 0;
    HAL_UART_TxCpltCallback(h);
    assert(MrLink_Channel_Send(&channel, source, 3) == MRLINK_CHANNEL_OK);
    h->ErrorCode = HAL_UART_ERROR_DMA;
    abort_fail = true;
    h->RxState = 0;
    HAL_UART_ErrorCallback(h);
    assert(MrLink_Channel_Send(&channel, source, 3) == MRLINK_CHANNEL_ERR_BUSY);
    assert(MrLink_Channel_Deinit(&channel) != MRLINK_CHANNEL_OK &&
           channel.hardware == MRLINK_CHANNEL_HW_UART);
    abort_fail = false;
    assert(MrLink_Channel_Deinit(&channel) == MRLINK_CHANNEL_OK);
    h->ErrorCode = 0;
    HAL_UART_TxCpltCallback(h);  // detached callback must be harmless
    assert(MrLink_Channel_InitUart(&other, &cfg) == MRLINK_CHANNEL_OK);
    assert(MrLink_Channel_Deinit(&other) == MRLINK_CHANNEL_OK);
  }
  UART_HandleTypeDef unknown = {(void*)99, &dma, 0, 0, 0, 0};
  BSP_UART_IRQHandler(&unknown);
  BSP_UART_IRQHandler(NULL);
  assert(BSP_UART_RegisterCallback((BSP_UART_t)-1, BSP_UART_TX_CPLT_CB, Legacy) != BSP_OK);
}
static MrLink_ChannelNotify_t usb_done, usb_error;
static void* usb_callback_ctx;
static bool usb_stop_fail, usb_register_fail;
static int8_t UsbSend(void* ctx, uint8_t* p, uint16_t n) {
  (void)ctx;
  (void)n;
  hal_tx = p;
  return 0;
}
static uint16_t UsbPop(void* ctx, uint8_t* p, uint16_t n) {
  (void)ctx;
  (void)p;
  (void)n;
  return 0;
}
static int8_t UsbCallbacks(void* ctx, MrLink_ChannelNotify_t d, MrLink_ChannelNotify_t e, void* c) {
  (void)ctx;
  if (usb_register_fail) return -1;
  usb_done = d;
  usb_error = e;
  usb_callback_ctx = c;
  return 0;
}
static int8_t UsbStop(void* ctx) {
  (void)ctx;
  if (usb_stop_fail) return -1;
  usb_done = NULL;
  usb_error = NULL;
  return 0;
}
static void TestUsb(void) {
  MrLink_Channel_t ch = {0};
  MrLink_ChannelUsbConfig_t cfg = {0};
  cfg.send = UsbSend;
  cfg.pop_rx = UsbPop;
  assert(MrLink_Channel_InitUsb(&ch, &cfg) == MRLINK_CHANNEL_ERR_UNSUPPORTED);
  cfg.register_tx_callbacks = UsbCallbacks;
  cfg.stop = UsbStop;
  usb_register_fail = true;
  assert(MrLink_Channel_InitUsb(&ch, &cfg) != 0 && ch.hardware == MRLINK_CHANNEL_HW_NONE);
  usb_register_fail = false;
  assert(MrLink_Channel_InitUsb(&ch, &cfg) == 0);
  uint8_t data[] = {1, 2, 3};
  assert(MrLink_Channel_Send(&ch, data, 3) == 0);
  data[0] = 9;
  assert(hal_tx[0] == 1);
  assert(MrLink_Channel_Send(&ch, data, 3) == MRLINK_CHANNEL_ERR_BUSY);
  usb_done(usb_callback_ctx);
  assert(MrLink_Channel_Send(&ch, data, 3) == 0);
  usb_error(usb_callback_ctx);
  assert(MrLink_Channel_Send(&ch, data, 3) == 0);
  usb_stop_fail = true;
  assert(MrLink_Channel_Deinit(&ch) != 0 && usb_done);
  usb_stop_fail = false;
  assert(MrLink_Channel_Deinit(&ch) == 0 && !usb_done);
  cfg.synchronous_send = true;
  assert(MrLink_Channel_InitUsb(&ch, &cfg) == 0);
  assert(MrLink_Channel_RegisterTxCallbacks(&ch, Done, Error, NULL) == 0);
  const unsigned before = done_calls;
  assert(MrLink_Channel_Send(&ch, data, 3) == 0);
  assert(MrLink_Channel_Send(&ch, data, 3) == 0 && done_calls == before + 2);
  assert(MrLink_Channel_Deinit(&ch) == 0);
}
static void TestFdcan(void) {
  MrLink_Channel_t ch = {0}, duplicate = {0};
  MrLink_ChannelFdcanConfig_t cfg = {0};
  cfg.tx_id = 0x100;
  cfg.rx_id = 0x101;
  cfg.max_frame_size = 64;
  assert(MrLink_Channel_InitFdcan(&ch, &cfg) == 0);
  assert(MrLink_Channel_InitFdcan(&duplicate, &cfg) != 0 &&
         duplicate.hardware == MRLINK_CHANNEL_HW_NONE);
  fd_message.original_id = 0x101;
  fd_message.frame_type = BSP_FDCAN_FRAME_STD_DATA;
  fd_message.dlc = 8;
  for (unsigned i = 0; i < 8; ++i) fd_message.data[i] = (uint8_t)i;
  fd_has_message = true;
  uint8_t p[3];
  assert(MrLink_Channel_PopRx(&ch, p, 3) == 3 && p[2] == 2);
  assert(MrLink_Channel_PopRx(&ch, p, 3) == 3 && p[0] == 3);
  assert(MrLink_Channel_PopRx(&ch, p, 3) == 2 && p[1] == 7);
  fd_unregister_fail = true;
  assert(MrLink_Channel_Deinit(&ch) != 0 && fd_registered);
  fd_unregister_fail = false;
  assert(MrLink_Channel_Deinit(&ch) == 0 && !fd_registered);
  cfg.tx_id = 0x800;
  assert(MrLink_Channel_InitFdcan(&ch, &cfg) != 0);
}
static void TestSpi(void) {
  uint8_t p[8];
  spi_count = 0;
  assert(BSP_SPI_MemRead(BSP_SPI_BMI088, 1, p, 8) == BSP_OK);
  assert(spi_count == 2 && spi_trace[0] == 'T' && spi_trace[1] == 'R');
  spi_result = HAL_ERROR;
  spi_count = 0;
  assert(BSP_SPI_MemWrite(BSP_SPI_BMI088, 1, p, 8) != BSP_OK && spi_count == 1);
  spi_result = HAL_OK;
  assert(BSP_SPI_RegisterCallback(BSP_SPI_BMI088, BSP_SPI_RX_CPLT_CB, SpiDone) == BSP_OK);
  assert(BSP_SPI_RegisterCallback(BSP_SPI_BMI088, BSP_SPI_ERROR_CB, SpiError) == BSP_OK);
  assert(BSP_SPI_Receive(BSP_SPI_BMI088, p, 8, true) == BSP_OK);
  unsigned before = done_calls;
  HAL_SPI_RxCpltCallback(&hspi2);
  assert(done_calls == before + 1);
  assert(BSP_SPI_Receive(BSP_SPI_BMI088, p, 8, true) == BSP_OK);
  before = done_calls;
  unsigned errors = error_calls;
  reject_cache = true;
  HAL_SPI_RxCpltCallback(&hspi2);
  assert(done_calls == before && error_calls == errors + 1);
  assert(BSP_SPI_Receive(BSP_SPI_BMI088, p, 8, true) != BSP_OK);
  reject_cache = false;
  hspi2.State = 1;
  unsigned sync_before = sync_calls;
  assert(BSP_SPI_Receive(BSP_SPI_BMI088, p, 8, true) != BSP_OK && sync_calls == sync_before);
  hspi2.State = 0;
  assert(BSP_SPI_RegisterCallback((BSP_SPI_t)-1, BSP_SPI_RX_CPLT_CB, SpiDone) != BSP_OK);
}
static void TestBmiDma(void) {
  BMI088_t imu;
  const BMI088_Cali_t cali = {{0, 0, 0}};
  assert(BMI088_Init(&imu, &cali) == DEVICE_OK);
  assert(BMI088_AcclStartDmaRecv() == DEVICE_OK);
  assert(((uintptr_t)hal_rx % 32u) == 0u);
  assert(BMI088_GyroStartDmaRecv() != DEVICE_OK);
  memset(hal_rx, 0, 19);
  HAL_SPI_RxCpltCallback(&hspi2);
  assert((BMI088_AcclWaitDmaCplt(0) & SIGNAL_BMI088_ACCL_RAW_REDY) != 0);
  assert(BSP_GPIO_ReadPin(BSP_GPIO_ACCL_CS));
  assert(BMI088_GyroStartDmaRecv() == DEVICE_OK);
  assert(((uintptr_t)hal_rx % 32u) == 0u);
  memset(hal_rx, 0, 6);
  HAL_SPI_RxCpltCallback(&hspi2);
  assert((BMI088_GyroWaitDmaCplt(0) & SIGNAL_BMI088_GYRO_RAW_REDY) != 0);
  assert(BMI088_ParseGyro(&imu) == DEVICE_OK && imu.gyro.x == 0);
  assert(BMI088_AcclStartDmaRecv() == DEVICE_OK);
  assert(BMI088_AcclWaitDmaCplt(0) == osFlagsErrorResource);
  assert(BSP_GPIO_ReadPin(BSP_GPIO_ACCL_CS));
  assert(BMI088_GyroStartDmaRecv() == DEVICE_OK);
  spi_result = HAL_ERROR;
  assert(BMI088_GyroWaitDmaCplt(0) == osFlagsErrorResource);
  assert(BMI088_AcclStartDmaRecv() != DEVICE_OK);  // DMA still owns storage on failed abort
  spi_result = HAL_OK;
  HAL_SPI_RxCpltCallback(&hspi2);
  assert(BMI088_AcclStartDmaRecv() == DEVICE_OK);
  reject_cache = true;
  HAL_SPI_RxCpltCallback(&hspi2);
  reject_cache = false;
  assert(BMI088_AcclWaitDmaCplt(0) ==
         osFlagsErrorResource);  // failed invalidation never publishes data
  reject_cache = true;
  assert(BMI088_GyroStartDmaRecv() != DEVICE_OK);
  reject_cache = false;
  assert(BSP_GPIO_ReadPin(BSP_GPIO_GYRO_CS));
}
static void TestDr16(void) {
  DR16_t dr16;
  assert(DR16_Init(&dr16) == DEVICE_OK);
  assert(DR16_StartDmaRecv(&dr16) == DEVICE_OK && (uintptr_t)hal_rx % 32u == 0u);
  assert(DR16_StartDmaRecv(&dr16) != DEVICE_OK);
  DR16_RawData_t raw = {0};
  raw.ch_r_x = raw.ch_r_y = raw.ch_l_x = raw.ch_l_y = 1024;
  raw.sw_l = raw.sw_r = 1;
  memcpy(hal_rx, &raw, sizeof(raw));
  huart5.RxState = 0;
  HAL_UARTEx_RxEventCallback(&huart5, sizeof(raw));
  assert(DR16_WaitDmaCplt(0) && DR16_ParseData(&dr16) == DEVICE_OK);
  assert(DR16_StartDmaRecv(&dr16) == DEVICE_OK);
  assert(!DR16_WaitDmaCplt(0) && huart5.RxState == HAL_UART_STATE_READY);
  assert(DR16_StartDmaRecv(&dr16) == DEVICE_OK);
  abort_fail = true;
  assert(DR16_Deinit() != DEVICE_OK);
  abort_fail = false;
  assert(DR16_Deinit() == DEVICE_OK);
  assert(BSP_UART_RegisterCallback(BSP_UART_RC, BSP_UART_TX_CPLT_CB, Legacy) == BSP_OK);
  assert(BSP_UART_RegisterCallback(BSP_UART_RC, BSP_UART_TX_CPLT_CB, NULL) == BSP_OK);
}
int main(void) {
  uint8_t data[8];
  assert(BSP_UART_Transmit(BSP_UART_PC, data, 8, true) != BSP_OK && tx_starts == 0);
  assert(BSP_DMA_Configure(Cache, NULL));
  TestUart();
  TestUsb();
  TestFdcan();
  TestSpi();
  TestBmiDma();
  TestDr16();
  assert(io_primask == 0);
  puts("MRLink channel + actual UART/SPI BSP lifecycle tests passed");
}

/*
     BMI088 陀螺仪+加速度计传感器。
*/

#include "bmi088.h"

#include <cmsis_os2.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "bsp/time.h"
#include "bsp/gpio.h"
#include "bsp/spi.h"
#include "component/math/scalar.h"

#define BMI088_REG_ACCL_CHIP_ID (0x00)
#define BMI088_REG_ACCL_ERR (0x02)
#define BMI088_REG_ACCL_STATUS (0x03)
#define BMI088_REG_ACCL_X_LSB (0x12)
#define BMI088_REG_ACCL_X_MSB (0x13)
#define BMI088_REG_ACCL_Y_LSB (0x14)
#define BMI088_REG_ACCL_Y_MSB (0x15)
#define BMI088_REG_ACCL_Z_LSB (0x16)
#define BMI088_REG_ACCL_Z_MSB (0x17)
#define BMI088_REG_ACCL_SENSORTIME_0 (0x18)
#define BMI088_REG_ACCL_SENSORTIME_1 (0x19)
#define BMI088_REG_ACCL_SENSORTIME_2 (0x1A)
#define BMI088_REG_ACCL_INT_STAT_1 (0x1D)
#define BMI088_REG_ACCL_TEMP_MSB (0x22)
#define BMI088_REG_ACCL_TEMP_LSB (0x23)
#define BMI088_REG_ACCL_CONF (0x40)
#define BMI088_REG_ACCL_RANGE (0x41)
#define BMI088_REG_ACCL_INT1_IO_CONF (0x53)
#define BMI088_REG_ACCL_INT2_IO_CONF (0x54)
#define BMI088_REG_ACCL_INT1_INT2_MAP_DATA (0x58)
#define BMI088_REG_ACCL_SELF_TEST (0x6D)
#define BMI088_REG_ACCL_PWR_CONF (0x7C)
#define BMI088_REG_ACCL_PWR_CTRL (0x7D)
#define BMI088_REG_ACCL_SOFTRESET (0x7E)

#define BMI088_REG_GYRO_CHIP_ID (0x00)
#define BMI088_REG_GYRO_X_LSB (0x02)
#define BMI088_REG_GYRO_X_MSB (0x03)
#define BMI088_REG_GYRO_Y_LSB (0x04)
#define BMI088_REG_GYRO_Y_MSB (0x05)
#define BMI088_REG_GYRO_Z_LSB (0x06)
#define BMI088_REG_GYRO_Z_MSB (0x07)
#define BMI088_REG_GYRO_INT_STAT_1 (0x0A)
#define BMI088_REG_GYRO_RANGE (0x0F)
#define BMI088_REG_GYRO_BANDWIDTH (0x10)
#define BMI088_REG_GYRO_LPM1 (0x11)
#define BMI088_REG_GYRO_SOFTRESET (0x14)
#define BMI088_REG_GYRO_INT_CTRL (0x15)
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF (0x16)
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP (0x18)
#define BMI088_REG_GYRO_SELF_TEST (0x3C)

#define BMI088_CHIP_ID_ACCL (0x1E)
#define BMI088_CHIP_ID_GYRO (0x0F)

#define BMI088_LEN_RX_BUFF (19)
#define BMI088_SIGNAL_ERROR_MASK (0x80000000u)
#define BMI088_ACCL_NSS_SET() \
  BSP_GPIO_WritePin(hardware.accl_cs, GPIO_PIN_SET)
#define BMI088_ACCL_NSS_RESET() \
  BSP_GPIO_WritePin(hardware.accl_cs, GPIO_PIN_RESET)

#define BMI088_GYRO_NSS_SET() \
  BSP_GPIO_WritePin(hardware.gyro_cs, GPIO_PIN_SET)
#define BMI088_GYRO_NSS_RESET() \
  BSP_GPIO_WritePin(hardware.gyro_cs, GPIO_PIN_RESET)

typedef enum {
  BMI_ACCL,
  BMI_GYRO,
} BMI_Device_t;

static uint8_t buffer[2];
static uint8_t bmi088_rxbuf[BMI088_LEN_RX_BUFF];
_Alignas(32) static uint8_t bmi088_dma_rx[32];
static uint8_t* dma_destination;
static uint8_t dma_length;
static volatile bool dma_busy;
static BMI_Device_t dma_device;
#define BMI088_SIGNAL_IO_ERROR (1u << 30u)

static osThreadId_t thread_alert;
static bool inited = false;
static bool init_locked; // Failed HAL abort retains callbacks until reset.
static BMI088_Config_t hardware;

static BSP_GPIO_t BMI_ChipSelect(BMI_Device_t dv) {
  return dv == BMI_ACCL ? hardware.accl_cs : hardware.gyro_cs;
}
static bool BMI_WriteSingle(BMI_Device_t dv, uint8_t reg, uint8_t data) {
  if (BSP_TIME_Delay(1) != BSP_OK) return false;
  buffer[0] = (uint8_t)(reg & 0x7f);
  buffer[1] = data;
  if (BSP_GPIO_WritePin(BMI_ChipSelect(dv), false) != BSP_OK) return false;
  const int8_t result = BSP_SPI_Transmit(hardware.spi, buffer, 2u, false);
  const int8_t deselect = BSP_GPIO_WritePin(BMI_ChipSelect(dv), true);
  return result == BSP_OK && deselect == BSP_OK;
}
static bool BMI_ReadSingle(BMI_Device_t dv, uint8_t reg, uint8_t* value) {
  if (BSP_TIME_Delay(1) != BSP_OK) return false;
  buffer[0] = (uint8_t)(reg | 0x80);
  if (BSP_GPIO_WritePin(BMI_ChipSelect(dv), false) != BSP_OK) return false;
  const bool ok = BSP_SPI_Transmit(hardware.spi, buffer, 1u, false) == BSP_OK &&
                  BSP_SPI_Receive(hardware.spi, buffer, dv == BMI_ACCL ? 2u : 1u, false) == BSP_OK;
  const int8_t deselect = BSP_GPIO_WritePin(BMI_ChipSelect(dv), true);
  if (ok && deselect == BSP_OK) {
    *value = buffer[dv == BMI_ACCL ? 1 : 0];
    return true;
  }
  return false;
}

static bool BMI_Read(BMI_Device_t dv, uint8_t reg, uint8_t* data, uint8_t len) {
  if (!data || !len || len > sizeof(bmi088_dma_rx) || dma_busy || !inited ||
      osThreadGetId() != thread_alert) return false;
  osThreadFlagsClear(SIGNAL_BMI088_ACCL_RAW_REDY | SIGNAL_BMI088_GYRO_RAW_REDY |
                     BMI088_SIGNAL_IO_ERROR);
  dma_destination = data;
  dma_length = len;
  dma_device = dv;
  dma_busy = true;
  if (dv == BMI_ACCL)
    BMI088_ACCL_NSS_RESET();
  else
    BMI088_GYRO_NSS_RESET();
  buffer[0] = (uint8_t)(reg | 0x80);
  if (BSP_SPI_Transmit(hardware.spi, buffer, 1u, false) == BSP_OK &&
      BSP_SPI_Receive(hardware.spi, bmi088_dma_rx, len, true) == BSP_OK)
    return true;
  dma_busy = false;
  dma_destination = NULL;
  BMI088_ACCL_NSS_SET();
  BMI088_GYRO_NSS_SET();
  return false;
}
static void BMI088_RxCpltCallback(void) {
  if (!dma_busy || !dma_destination) return;
  memcpy(dma_destination, bmi088_dma_rx, dma_length);
  dma_destination = NULL;
  dma_busy = false;
  BMI088_ACCL_NSS_SET();
  BMI088_GYRO_NSS_SET();
  osThreadFlagsSet(thread_alert, dma_device == BMI_ACCL ? SIGNAL_BMI088_ACCL_RAW_REDY
                                                        : SIGNAL_BMI088_GYRO_RAW_REDY);
}
static void BMI088_ErrorCallback(void) {
  if (dma_busy) osThreadFlagsSet(thread_alert, BMI088_SIGNAL_IO_ERROR);
}
static uint32_t BMI088_WaitDma(uint32_t ready, uint32_t timeout_ms) {
  if (!inited || osThreadGetId() != thread_alert) return osFlagsErrorResource;
  const uint32_t flags =
      osThreadFlagsWait(ready | BMI088_SIGNAL_IO_ERROR, osFlagsWaitAny, timeout_ms);
  if ((flags & BMI088_SIGNAL_ERROR_MASK) == 0u && (flags & BMI088_SIGNAL_IO_ERROR) == 0u &&
      (flags & ready) != 0u)
    return flags;
  if (BSP_SPI_Abort(hardware.spi) == BSP_OK) {
    dma_busy = false;
    dma_destination = NULL;
    BMI088_ACCL_NSS_SET();
    BMI088_GYRO_NSS_SET();
  }
  return osFlagsErrorResource;
}

static void BMI088_AcclIntCallback(void) {
  osThreadFlagsSet(thread_alert, SIGNAL_BMI088_ACCL_NEW_DATA);
}

static void BMI088_GyroIntCallback(void) {
  osThreadFlagsSet(thread_alert, SIGNAL_BMI088_GYRO_NEW_DATA);
}

static bool BMI088_ValidPin(BSP_GPIO_t pin, BSP_GPIO_Mode_t mode) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(pin);
  return p && p->mode == mode;
}
int8_t BMI088_Init(BMI088_t *bmi088, const BMI088_Cali_t *cali,
                   const BMI088_Config_t *config) {
  if (!bmi088 || !cali || !config) return DEVICE_ERR_NULL;
  if (inited || init_locked) return DEVICE_ERR_INITED;
  bmi088->header.online = false;
  if (!BSP_SPI_GetHandle(config->spi) ||
      !BMI088_ValidPin(config->accl_cs, BSP_GPIO_OUTPUT) ||
      !BMI088_ValidPin(config->gyro_cs, BSP_GPIO_OUTPUT) ||
      !BMI088_ValidPin(config->accl_int, BSP_GPIO_EXTI) ||
      !BMI088_ValidPin(config->gyro_int, BSP_GPIO_EXTI) ||
      config->accl_cs == config->gyro_cs || config->accl_int == config->gyro_int ||
      !isfinite(cali->gyro_offset.x) || !isfinite(cali->gyro_offset.y) ||
      !isfinite(cali->gyro_offset.z)) return DEVICE_ERR;
  const osThreadId_t caller = osThreadGetId();
  if (!caller) return DEVICE_ERR_NULL;

  // Acquire callbacks without replacing or disabling someone else's resources.
  uint8_t claimed = 0;
  int8_t result = DEVICE_ERR;
  hardware = *config;
  thread_alert = caller;
  if (BSP_SPI_RegisterCallback(hardware.spi, BSP_SPI_RX_CPLT_CB,
                               BMI088_RxCpltCallback) != BSP_OK) goto fail;
  claimed |= 1u;
  if (BSP_SPI_RegisterCallback(hardware.spi, BSP_SPI_ERROR_CB,
                               BMI088_ErrorCallback) != BSP_OK) goto fail;
  claimed |= 2u;
  if (BSP_GPIO_RegisterCallback(hardware.accl_int, BMI088_AcclIntCallback) != BSP_OK) goto fail;
  claimed |= 4u;
  if (BSP_GPIO_RegisterCallback(hardware.gyro_int, BMI088_GyroIntCallback) != BSP_OK) goto fail;
  claimed |= 8u;

  if (BMI088_ACCL_NSS_SET() != BSP_OK || BMI088_GYRO_NSS_SET() != BSP_OK ||
      !BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_SOFTRESET, 0xB6) ||
      !BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_SOFTRESET, 0xB6) ||
      BSP_TIME_Delay(30) != BSP_OK) goto fail;
  uint8_t id;
  // The first accelerometer read switches it into SPI mode (dummy byte included).
  if (!BMI_ReadSingle(BMI_ACCL, BMI088_REG_ACCL_CHIP_ID, &id) ||
      !BMI_ReadSingle(BMI_ACCL, BMI088_REG_ACCL_CHIP_ID, &id)) goto fail;
  if (id != BMI088_CHIP_ID_ACCL) { result = DEVICE_ERR_NO_DEV; goto fail; }
  if (!BMI_ReadSingle(BMI_GYRO, BMI088_REG_GYRO_CHIP_ID, &id)) goto fail;
  if (id != BMI088_CHIP_ID_GYRO) { result = DEVICE_ERR_NO_DEV; goto fail; }

  // Accelerometer: 400 Hz, +/-6 g; INT1 push-pull, active low.
  if (!BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_CONF, 0xAA) ||
      !BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_RANGE, 0x01) ||
      !BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_INT1_IO_CONF, 0x08) ||
      !BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_INT1_INT2_MAP_DATA, 0x04) ||
      !BMI_WriteSingle(BMI_ACCL, BMI088_REG_ACCL_PWR_CTRL, 0x04) ||
      BSP_TIME_Delay(50) != BSP_OK) goto fail;
  // Gyroscope: +/-1000 deg/s, 400 Hz, 47 Hz filter; INT3 active low.
  if (!BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_RANGE, 0x01) ||
      !BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_BANDWIDTH, 0x03) ||
      !BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_INT3_INT4_IO_CONF, 0x00) ||
      !BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_INT3_INT4_IO_MAP, 0x01) ||
      !BMI_WriteSingle(BMI_GYRO, BMI088_REG_GYRO_INT_CTRL, 0x80) ||
      BSP_TIME_Delay(10) != BSP_OK) goto fail;

  bmi088->cali = cali;
  inited = true;
  if (BSP_GPIO_EnableIRQ(hardware.accl_int) != BSP_OK ||
      BSP_GPIO_EnableIRQ(hardware.gyro_int) != BSP_OK) goto fail;
  return DEVICE_OK;

fail:
  inited = false;
  if (claimed & 4u) BSP_GPIO_DisableIRQ(hardware.accl_int);
  if (claimed & 8u) BSP_GPIO_DisableIRQ(hardware.gyro_int);
  // Initialization never starts DMA. If HAL cannot recover an idle bus, retain
  // callbacks rather than letting another consumer attach to an uncertain bus.
  if ((claimed & 3u) && BSP_SPI_GetHandle(hardware.spi)->State != HAL_SPI_STATE_READY &&
      BSP_SPI_Abort(hardware.spi) != BSP_OK) {
    init_locked = true;
    return result;
  }
  if (claimed & 8u) BSP_GPIO_UnregisterCallback(hardware.gyro_int, BMI088_GyroIntCallback);
  if (claimed & 4u) BSP_GPIO_UnregisterCallback(hardware.accl_int, BMI088_AcclIntCallback);
  if (claimed & 2u) BSP_SPI_UnregisterCallback(hardware.spi, BSP_SPI_ERROR_CB, BMI088_ErrorCallback);
  if (claimed & 1u) BSP_SPI_UnregisterCallback(hardware.spi, BSP_SPI_RX_CPLT_CB, BMI088_RxCpltCallback);
  if (claimed == 15u) {
    BMI088_ACCL_NSS_SET();
    BMI088_GYRO_NSS_SET();
  }
  thread_alert = NULL;
  return result;
}

bool BMI088_GyroStable(AHRS_Gyro_t *gyro) {
  if (gyro == NULL || !isfinite(gyro->x) || !isfinite(gyro->y) ||
      !isfinite(gyro->z)) {
    return false;
  }
  return ((fabsf(gyro->x) < 0.03f) && (fabsf(gyro->y) < 0.03f) &&
          (fabsf(gyro->z) < 0.03f));
}

static bool BMI088_FlagsOk(uint32_t flags, uint32_t required_flags) {
  return (flags & BMI088_SIGNAL_ERROR_MASK) == 0u &&
     (flags & required_flags) == required_flags;
}

uint32_t BMI088_WaitNew(uint32_t timeout_ms) {
  if (!inited || osThreadGetId() != thread_alert) return osFlagsErrorResource;
  return osThreadFlagsWait(
      SIGNAL_BMI088_ACCL_NEW_DATA | SIGNAL_BMI088_GYRO_NEW_DATA, osFlagsWaitAll,
  timeout_ms);
}

int8_t BMI088_AcclStartDmaRecv() {
  return BMI_Read(BMI_ACCL, BMI088_REG_ACCL_X_LSB, bmi088_rxbuf, BMI088_LEN_RX_BUFF) ? DEVICE_OK
                                                                                     : DEVICE_ERR;
}

uint32_t BMI088_AcclWaitDmaCplt(uint32_t timeout_ms) {
  return BMI088_WaitDma(SIGNAL_BMI088_ACCL_RAW_REDY, timeout_ms);
}

int8_t BMI088_GyroStartDmaRecv() {
  return BMI_Read(BMI_GYRO, BMI088_REG_GYRO_X_LSB, bmi088_rxbuf + 7, 6u) ? DEVICE_OK : DEVICE_ERR;
}

uint32_t BMI088_GyroWaitDmaCplt(uint32_t timeout_ms) {
  return BMI088_WaitDma(SIGNAL_BMI088_GYRO_RAW_REDY, timeout_ms);
}

bool BMI088_ReadFrameNonBlocking(uint32_t timeout_ms) {
  if (!BMI088_FlagsOk(BMI088_WaitNew(timeout_ms),
                      SIGNAL_BMI088_ACCL_NEW_DATA |
                          SIGNAL_BMI088_GYRO_NEW_DATA)) {
    return false;
  }

  if (BMI088_AcclStartDmaRecv() != DEVICE_OK) {
    return false;
  }
  if (!BMI088_FlagsOk(BMI088_AcclWaitDmaCplt(timeout_ms),
                      SIGNAL_BMI088_ACCL_RAW_REDY)) {
    return false;
  }

  if (BMI088_GyroStartDmaRecv() != DEVICE_OK) {
    return false;
  }
  if (!BMI088_FlagsOk(BMI088_GyroWaitDmaCplt(timeout_ms),
                      SIGNAL_BMI088_GYRO_RAW_REDY)) {
    return false;
  }

  return true;
}

int8_t BMI088_ParseAccl(BMI088_t *bmi088) {
  if (bmi088 == NULL) return DEVICE_ERR_NULL;

  int16_t raw_x, raw_y, raw_z;
  memcpy(&raw_x, bmi088_rxbuf + 1, sizeof(raw_x));
  memcpy(&raw_y, bmi088_rxbuf + 3, sizeof(raw_y));
  memcpy(&raw_z, bmi088_rxbuf + 5, sizeof(raw_z));

  bmi088->accl.x = (float)raw_x;
  bmi088->accl.y = (float)raw_y;
  bmi088->accl.z = (float)raw_z;

  /* 3G: 10920. 6G: 5460. 12G: 2730. 24G: 1365. */
  bmi088->accl.x /= 5460.0f;
  bmi088->accl.y /= 5460.0f;
  bmi088->accl.z /= 5460.0f;

  int16_t raw_temp =
      (uint16_t)((bmi088_rxbuf[17] << 3) | (bmi088_rxbuf[18] >> 5));

  if (raw_temp > 1023) raw_temp -= 2048;

  bmi088->temp = (float)raw_temp * 0.125f + 23.0f;

  return DEVICE_OK;
}

int8_t BMI088_ParseGyro(BMI088_t *bmi088) {
  if (bmi088 == NULL || bmi088->cali == NULL) return DEVICE_ERR_NULL;

  /* Gyroscope imu_raw -> degrees/sec -> radians/sec */
  int16_t raw_x, raw_y, raw_z;
  memcpy(&raw_x, bmi088_rxbuf + 7, sizeof(raw_x));
  memcpy(&raw_y, bmi088_rxbuf + 9, sizeof(raw_y));
  memcpy(&raw_z, bmi088_rxbuf + 11, sizeof(raw_z));

  bmi088->gyro.x = (float)raw_x;
  bmi088->gyro.y = (float)raw_y;
  bmi088->gyro.z = (float)raw_z;

  /* FS125: 262.144. FS250: 131.072. FS500: 65.536. FS1000: 32.768.
   * FS2000: 16.384.*/
  bmi088->gyro.x /= 32.768f;
  bmi088->gyro.y /= 32.768f;
  bmi088->gyro.z /= 32.768f;

  bmi088->gyro.x *= MR_MATH_DEG_TO_RAD;
  bmi088->gyro.y *= MR_MATH_DEG_TO_RAD;
  bmi088->gyro.z *= MR_MATH_DEG_TO_RAD;

  bmi088->gyro.x -= bmi088->cali->gyro_offset.x;
  bmi088->gyro.y -= bmi088->cali->gyro_offset.y;
  bmi088->gyro.z -= bmi088->cali->gyro_offset.z;

  return DEVICE_OK;
}

float BMI088_GetUpdateFreq(BMI088_t *bmi088) {
  (void)bmi088;
  return 400.0f;
}

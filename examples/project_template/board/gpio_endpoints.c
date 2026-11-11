#include "board/gpio_endpoints.h"
#include "bsp/gpio.h"
#include "main.h"

int Board_RegisterGpioEndpoints(void) {
  // 使用 Cube main.h 的引脚名；这里只登记，不改变电平、模式和 EXTI 边沿。
  // EXTI 登记后保持屏蔽，设备安装回调后显式 EnableIRQ。
  const BSP_GPIO_Endpoint_t endpoints[] = {
      {"accel_cs", ACCL_CS_GPIO_Port, ACCL_CS_Pin, BSP_GPIO_OUTPUT},
      {"gyro_cs", GYRO_CS_GPIO_Port, GYRO_CS_Pin, BSP_GPIO_OUTPUT},
      {"accel_int", ACCL_INT_GPIO_Port, ACCL_INT_Pin, BSP_GPIO_EXTI},
      {"gyro_int", GYRO_INT_GPIO_Port, GYRO_INT_Pin, BSP_GPIO_EXTI},
  };
  return BSP_GPIO_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}

// Hardware inventory for this H723 project. Cube owns GPIO/SPI initialization.
#include "bsp/gpio.h"
#include "bsp/spi.h"
#include "main.h"
#include "spi.h"

int Board_RegisterIoEndpoints(void) {
  // BMI088 driver configures active-low data-ready outputs. Override the source
  // Cube project's rising edges in THIS example only; never edit external Cube.
  GPIO_InitTypeDef interrupts = {0};
  interrupts.Pin = ACCL_INT_Pin | GYRO_INT_Pin;
  interrupts.Mode = GPIO_MODE_IT_FALLING;
  interrupts.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &interrupts);
  const BSP_GPIO_Endpoint_t pins[] = {
      {"pc0", ACCL_CS_GPIO_Port, ACCL_CS_Pin, BSP_GPIO_OUTPUT},
      {"pc3", GYRO_CS_GPIO_Port, GYRO_CS_Pin, BSP_GPIO_OUTPUT},
      {"pe10", ACCL_INT_GPIO_Port, ACCL_INT_Pin, BSP_GPIO_EXTI},
      {"pe12", GYRO_INT_GPIO_Port, GYRO_INT_Pin, BSP_GPIO_EXTI},
      {"pa15", USER_KEY_GPIO_Port, USER_KEY_Pin, BSP_GPIO_EXTI},
      {"pc13", Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin, BSP_GPIO_OUTPUT},
      {"pc14", Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, BSP_GPIO_OUTPUT},
      {"pc15", Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, BSP_GPIO_OUTPUT},
      {"pc10", valve_arm_GPIO_Port, valve_arm_Pin, BSP_GPIO_OUTPUT},
      {"pc11", valve_rod_GPIO_Port, valve_rod_Pin, BSP_GPIO_OUTPUT},
      {"pa2", ore_relay_GPIO_Port, ore_relay_Pin, BSP_GPIO_OUTPUT},
      {"pa0", spearhead_relay_GPIO_Port, spearhead_relay_Pin, BSP_GPIO_OUTPUT},
      {"pe14", checkphoto_spear_GPIO_Port, checkphoto_spear_Pin, BSP_GPIO_INPUT},
  };
  const BSP_SPI_Endpoint_t buses[] = {{"spi2", &hspi2}};
  return BSP_GPIO_Configure(pins, sizeof(pins) / sizeof(pins[0])) == BSP_OK &&
         BSP_SPI_Configure(buses, sizeof(buses) / sizeof(buses[0])) == BSP_OK;
}

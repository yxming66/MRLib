#include "bsp/gpio.h"
#include <stddef.h>
#include <string.h>

static BSP_GPIO_Endpoint_t endpoints[BSP_GPIO_NUM];
static void (*callbacks[BSP_GPIO_NUM])(void);
static uint8_t endpoint_count;
static bool configured;

const BSP_GPIO_Endpoint_t* BSP_GPIO_GetEndpoint(BSP_GPIO_t gpio) {
  return (unsigned)gpio < endpoint_count ? &endpoints[gpio] : NULL;
}
int8_t BSP_GPIO_Configure(const BSP_GPIO_Endpoint_t* table, uint8_t count) {
  if (configured || !table || !count || count > BSP_GPIO_NUM) return BSP_ERR;
  uint32_t exti_mask = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const BSP_GPIO_Endpoint_t* p = &table[i];
    if (!p->name || !p->name[0] || !p->port || !p->pin ||
        (p->pin & (p->pin - 1u)) != 0 || (unsigned)p->mode > BSP_GPIO_EXTI)
      return BSP_ERR;
    if (p->mode == BSP_GPIO_EXTI) {
      if (exti_mask & p->pin) return BSP_ERR; // EXTI line cannot route two ports.
      exti_mask |= p->pin;
    }
    for (uint8_t j = 0; j < i; ++j)
      if (strcmp(p->name, table[j].name) == 0 ||
          (p->port == table[j].port && p->pin == table[j].pin)) return BSP_ERR;
  }
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  EXTI_D1->IMR1 &= ~exti_mask;
  __HAL_GPIO_EXTI_CLEAR_IT(exti_mask);
  memcpy(endpoints, table, count * sizeof(*table));
  endpoint_count = count;
  configured = true;
  if (!token) __enable_irq();
  return BSP_OK;
}
BSP_GPIO_t BSP_GPIO_FindEndpoint(const char* name) {
  if (name)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (strcmp(name, endpoints[i].name) == 0) return (BSP_GPIO_t)i;
  return BSP_GPIO_ERR;
}
int8_t BSP_GPIO_RegisterCallback(BSP_GPIO_t gpio, void (*callback)(void)) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_EXTI || !callback) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (!(EXTI_D1->IMR1 & p->pin) && !callbacks[gpio]) {
    callbacks[gpio] = callback;
    result = BSP_OK;
  }
  if (!token) __enable_irq();
  return result;
}
int8_t BSP_GPIO_UnregisterCallback(BSP_GPIO_t gpio, void (*callback)(void)) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_EXTI || !callback) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (!(EXTI_D1->IMR1 & p->pin) && callbacks[gpio] == callback) {
    callbacks[gpio] = NULL;
    result = BSP_OK;
  }
  if (!token) __enable_irq();
  return result;
}
int8_t BSP_GPIO_EnableIRQ(BSP_GPIO_t gpio) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_EXTI) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t result = BSP_ERR;
  if (callbacks[gpio]) {
    if (!(EXTI_D1->IMR1 & p->pin)) __HAL_GPIO_EXTI_CLEAR_IT(p->pin);
    EXTI_D1->IMR1 |= p->pin;
    result = BSP_OK;
  }
  if (!token) __enable_irq();
  return result;
}
int8_t BSP_GPIO_DisableIRQ(BSP_GPIO_t gpio) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_EXTI) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  EXTI_D1->IMR1 &= ~(uint32_t)p->pin;
  __HAL_GPIO_EXTI_CLEAR_IT(p->pin);
  if (!token) __enable_irq();
  return BSP_OK;
}
void HAL_GPIO_EXTI_Callback(uint16_t pins) {
  for (uint8_t i = 0; i < endpoint_count; ++i) {
    const BSP_GPIO_Endpoint_t* p = &endpoints[i];
    if (p->mode == BSP_GPIO_EXTI && (pins & p->pin) &&
        (EXTI_D1->IMR1 & p->pin) && callbacks[i]) callbacks[i]();
  }
}
int8_t BSP_GPIO_WritePin(BSP_GPIO_t gpio, bool value) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_OUTPUT) return BSP_ERR;
  HAL_GPIO_WritePin(p->port, p->pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return BSP_OK;
}
int8_t BSP_GPIO_TogglePin(BSP_GPIO_t gpio) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  if (!p || p->mode != BSP_GPIO_OUTPUT) return BSP_ERR;
  HAL_GPIO_TogglePin(p->port, p->pin);
  return BSP_OK;
}
bool BSP_GPIO_ReadPin(BSP_GPIO_t gpio) {
  const BSP_GPIO_Endpoint_t* p = BSP_GPIO_GetEndpoint(gpio);
  return p && HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET;
}

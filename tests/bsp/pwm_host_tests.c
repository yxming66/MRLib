// Real PWM and buzzer source, deterministic registers/HAL. No board htim globals.
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "bsp/pwm.h"
#include "device/buzzer.h"

static TIM_TypeDef solo_regs = {.ARR = 999}, shared_regs = {.PSC = 239, .ARR = 19999};
static TIM_TypeDef wide_regs = {.ARR = UINT32_MAX};
static TIM_HandleTypeDef solo = {&solo_regs, {0, 999, 0, 0}, HAL_TIM_STATE_READY};
static TIM_HandleTypeDef shared = {&shared_regs, {239, 19999, 0, 0}, HAL_TIM_STATE_READY};
static TIM_HandleTypeDef wide = {&wide_regs, {0, UINT32_MAX, 0, 0}, HAL_TIM_STATE_READY};
static TIM_HandleTypeDef* last_timer;
static uint32_t last_channel;
static unsigned starts, stops;
static HAL_StatusTypeDef hal_result = HAL_OK;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* t, uint32_t channel) {
  last_timer = t; last_channel = channel; ++starts; return hal_result;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* t, uint32_t channel) {
  last_timer = t; last_channel = channel; ++stops; return hal_result;
}
uint32_t BSP_TIME_Get_ms(void) { return 123; }

static void TestRegistration(void) {
  assert(BSP_PWM_GetHandle(BSP_PWM_0) == NULL);
  assert(BSP_PWM_FindEndpoint("tone") == BSP_PWM_ERR);
  assert(BSP_PWM_Start(BSP_PWM_0) == BSP_ERR && starts == 0);
  BSP_PWM_Endpoint_t invalid[] = {
      {"a", &solo, TIM_CHANNEL_1, 1000000, 16, false},
      {"a", &shared, TIM_CHANNEL_1, 240000000, 16, false}};
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR);
  assert(BSP_PWM_FindEndpoint("a") == BSP_PWM_ERR); // No partial publication.
  invalid[1].name = "b";
  invalid[1].timer = &solo;
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR); // Duplicate timer/channel.
  TIM_HandleTypeDef copied = solo;
  invalid[1].timer = &copied;
  invalid[1].channel = TIM_CHANNEL_2;
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR); // Aliased physical timer handle.
  invalid[1].timer = &solo;
  invalid[1].input_clock_hz = 1000000;
  invalid[1].counter_bits = 32;
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR); // Inconsistent timer width.
  invalid[1].counter_bits = 16;
  invalid[1].allow_frequency_change = true;
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR); // Retunable shared timer.
  invalid[1].allow_frequency_change = false;
  invalid[1].input_clock_hz = 2000000;
  assert(BSP_PWM_Configure(invalid, 2) == BSP_ERR);
  invalid[0].timer = NULL;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  invalid[0].timer = &solo;
  solo.State = 0;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  solo.State = HAL_TIM_STATE_READY;
  invalid[0].name = "";
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  invalid[0].name = "a";
  invalid[0].input_clock_hz = 0;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  invalid[0].input_clock_hz = 1000000;
  invalid[0].channel = 99;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  invalid[0].channel = TIM_CHANNEL_1;
  solo.Init.CounterMode = 1;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  solo.Init.CounterMode = 0;
  solo_regs.CR1 = TIM_CR1_CEN;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  solo_regs.CR1 = 0;
  invalid[0].counter_bits = 8;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  invalid[0].counter_bits = 16;
  solo_regs.ARR = UINT16_MAX + 1u;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  solo_regs.ARR = 999;
  solo_regs.PSC = UINT16_MAX + 1u;
  assert(BSP_PWM_Configure(invalid, 1) == BSP_ERR);
  solo_regs.PSC = 0;
  assert(BSP_PWM_Configure(NULL, 0) == BSP_ERR);
  assert(BSP_PWM_Configure(invalid, BSP_PWM_NUM + 1) == BSP_ERR);
  const BSP_PWM_Endpoint_t valid[] = {
      {"servo_b", &shared, TIM_CHANNEL_3, 240000000, 16, false},
      {"tone", &solo, TIM_CHANNEL_2, 1000000, 16, true},
      {"servo_a", &shared, TIM_CHANNEL_1, 240000000, 16, false},
      {"wide", &wide, TIM_CHANNEL_4, 240000000, 32, true}};
  assert(BSP_PWM_Configure(valid, 4) == BSP_OK);
  assert(BSP_PWM_Configure(valid, 4) == BSP_ERR);
  assert(BSP_PWM_FindEndpoint("tone") == BSP_PWM_1); // Order is not timer number.
  assert(BSP_PWM_FindEndpoint(NULL) == BSP_PWM_ERR);
  assert(BSP_PWM_FindEndpoint("missing") == BSP_PWM_ERR);
}

static void TestOutputs(void) {
  const BSP_PWM_Channel_t tone = BSP_PWM_FindEndpoint("tone");
  assert(BSP_PWM_Start(tone) == BSP_OK);
  assert(last_timer == &solo && last_channel == TIM_CHANNEL_2);
  hal_result = HAL_ERROR;
  assert(BSP_PWM_Start(tone) == BSP_ERR);
  assert(BSP_PWM_Stop(tone) == BSP_ERR);
  hal_result = HAL_OK;
  assert(BSP_PWM_Stop(tone) == BSP_OK);
  assert(BSP_PWM_SetComp(tone, 0.5f) == BSP_OK && solo_regs.CCR[1] == 500);
  assert(BSP_PWM_SetComp(tone, NAN) == BSP_ERR && solo_regs.CCR[1] == 500);
  assert(BSP_PWM_SetComp(tone, INFINITY) == BSP_ERR);
  assert(BSP_PWM_SetComp(tone, -0.5f) == BSP_OK && solo_regs.CCR[1] == 0);
  assert(BSP_PWM_SetComp(tone, 1.5f) == BSP_OK && solo_regs.CCR[1] == 1000);
  assert(BSP_PWM_SetComp(tone, 0.5f) == BSP_OK);
  assert(BSP_PWM_SetFreq(tone, 440.0f) == BSP_OK);
  assert(solo_regs.PSC == 0 && solo_regs.ARR == 2272 && solo_regs.CCR[1] == 1136);
  assert(solo.Init.Prescaler == solo_regs.PSC && solo.Init.Period == solo_regs.ARR);
  assert(solo_regs.CNT == 0 && solo_regs.EGR == TIM_EGR_UG);
  assert(BSP_PWM_SetFreq(tone, 1) == BSP_OK); // Requires a changed prescaler.
  assert(solo_regs.PSC == 15 && solo_regs.ARR == 62499);
  const TIM_TypeDef before = solo_regs;
  const float bad[] = {0, -1, NAN, INFINITY, 1000000, 1e-20f};
  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
    assert(BSP_PWM_SetFreq(tone, bad[i]) == BSP_ERR);
    assert(memcmp(&before, &solo_regs, sizeof(before)) == 0);
  }
  const BSP_PWM_Channel_t a = BSP_PWM_FindEndpoint("servo_a");
  const BSP_PWM_Channel_t b = BSP_PWM_FindEndpoint("servo_b");
  const TIM_TypeDef shared_before = shared_regs;
  assert(BSP_PWM_SetFreq(a, 50) == BSP_ERR);
  assert(BSP_PWM_SetFreq(b, 100) == BSP_ERR);
  assert(memcmp(&shared_before, &shared_regs, sizeof(shared_before)) == 0);
  assert(BSP_PWM_SetPulseUs(a, 1500) == BSP_OK && shared_regs.CCR[0] == 1500);
  assert(BSP_PWM_SetPulseUs(b, 2000) == BSP_OK && shared_regs.CCR[2] == 2000);
  assert(shared_regs.CCR[0] == 1500 && shared_regs.ARR == 19999);
  assert(BSP_PWM_SetPulseUs(b, UINT32_MAX) == BSP_OK && shared_regs.CCR[2] == 20000);
  assert(BSP_PWM_GetChannel((BSP_PWM_Channel_t)-1) == UINT32_MAX);
  assert(BSP_PWM_GetHandle(BSP_PWM_ERR) == NULL);
  assert(BSP_PWM_GetHandle(BSP_PWM_15) == NULL);
  assert(BSP_PWM_SetPulseUs(BSP_PWM_ERR, 100) == BSP_ERR);
  assert(BSP_PWM_SetComp((BSP_PWM_Channel_t)-1, 0) == BSP_ERR);
  const BSP_PWM_Channel_t w = BSP_PWM_FindEndpoint("wide");
  assert(BSP_PWM_SetComp(w, 1) == BSP_ERR); // ARR+1 must not wrap to zero.
  assert(BSP_PWM_SetComp(w, 0.5f) == BSP_OK && wide_regs.CCR[3] == 0x80000000u);
  assert(BSP_PWM_SetPulseUs(w, UINT32_MAX) == BSP_ERR);
  assert(BSP_PWM_SetFreq(w, 0.1f) == BSP_OK); // 32-bit counter, not 16-bit limited.
  assert(wide_regs.ARR > UINT16_MAX && wide_regs.PSC == 0);
  assert(BSP_PWM_SetComp(w, 1) == BSP_OK && wide_regs.CCR[3] == wide_regs.ARR + 1u);
}

static void TestBuzzer(void) {
  BUZZER_t buzzer = {0};
  assert(BUZZER_Init(&buzzer, BSP_PWM_ERR) == DEVICE_ERR && !buzzer.header.online);
  const BSP_PWM_Channel_t tone = BSP_PWM_FindEndpoint("tone");
  hal_result = HAL_ERROR;
  assert(BUZZER_Init(&buzzer, tone) == DEVICE_ERR && !buzzer.header.online);
  hal_result = HAL_OK;
  assert(BUZZER_Init(&buzzer, tone) == DEVICE_OK && buzzer.header.online);
  assert(last_timer == &solo && last_channel == TIM_CHANNEL_2);
  assert(BUZZER_Set(&buzzer, 440, 0.5f) == DEVICE_OK);
  const TIM_TypeDef before = solo_regs;
  assert(BUZZER_Set(&buzzer, 440, NAN) == DEVICE_ERR);
  assert(memcmp(&before, &solo_regs, sizeof(before)) == 0);
  assert(BUZZER_Start(&buzzer) == DEVICE_OK);
  hal_result = HAL_ERROR;
  assert(BUZZER_Stop(&buzzer) == DEVICE_ERR);
  hal_result = HAL_OK;
  assert(BUZZER_Stop(&buzzer) == DEVICE_OK);
  assert(BUZZER_Init(&buzzer, BSP_PWM_FindEndpoint("servo_a")) == DEVICE_OK);
  const TIM_TypeDef shared_before = shared_regs;
  assert(BUZZER_Set(&buzzer, 440, 0.5f) == DEVICE_ERR);
  assert(memcmp(&shared_before, &shared_regs, sizeof(shared_before)) == 0);
}
int main(void) {
  TestRegistration();
  TestOutputs();
  TestBuzzer();
  puts("PWM endpoint, shared timer and buzzer tests passed");
}

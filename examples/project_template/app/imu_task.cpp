#include "app/imu_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "board/bindings.hpp"
#include "bsp/time.h"
#include "device/bmi088.h"

namespace {

constexpr uint32_t kStackBytes = 2048;
constexpr uint32_t kWaitTicks = 10;  // 本模板 RTOS tick 为 1 ms。
constexpr float kStandardGravityMps2 = 9.80665f;
constexpr BMI088_Cali_t kGyroCalibration{};  // 实测静止偏置后按传感器坐标填 rad/s。

// 同时供调试器观察。任务代码通过 ReadLatestImu 读取完整快照。
app::ImuReading g_imu_latest{};

void StoreReading(const app::ImuReading& reading) {
  taskENTER_CRITICAL();
  g_imu_latest = reading;
  taskEXIT_CRITICAL();
}

// 1. 在采集线程中初始化。BMI088 中断会通知调用 Init 的线程。
void Task_Imu(void*) {
  BMI088_t imu{};
  const BMI088_Config_t config{
      BSP_SPI_FindEndpoint(board::bindings::kImuSpi),
      BSP_GPIO_FindEndpoint(board::bindings::kAccelCs),
      BSP_GPIO_FindEndpoint(board::bindings::kGyroCs),
      BSP_GPIO_FindEndpoint(board::bindings::kAccelInt),
      BSP_GPIO_FindEndpoint(board::bindings::kGyroInt)};
  app::ImuReading reading{};
  reading.init_result = BMI088_Init(&imu, &kGyroCalibration, &config);
  StoreReading(reading);
  if (reading.init_result != DEVICE_OK) {
    osThreadExit();
    return;
  }

  // 2. 等两路数据就绪，依次接收两次 SPI DMA，然后解析完整六轴帧。
  for (;;) {
    if (!BMI088_ReadFrameNonBlocking(kWaitTicks) ||
        BMI088_ParseAccl(&imu) != DEVICE_OK || BMI088_ParseGyro(&imu) != DEVICE_OK) {
      reading.sample.valid = false;
      ++reading.failed_frames;
      StoreReading(reading);
      (void)osDelay(1u);
      continue;
    }

    // 3. 统一单位并发布。采样时间取本板读完一帧的时刻。
    auto& sample = reading.sample;
    sample.acceleration_mps2 = {imu.accl.x * kStandardGravityMps2,
                                imu.accl.y * kStandardGravityMps2,
                                imu.accl.z * kStandardGravityMps2};
    sample.angular_velocity_rad_s = {imu.gyro.x, imu.gyro.y, imu.gyro.z};
    sample.timestamp_us = BSP_TIME_Get_us();
    ++sample.sequence;
    sample.valid = true;
    reading.temperature_c = imu.temp;
    StoreReading(reading);
  }
}

}  // namespace

namespace app {

bool CreateImuTask() {
  static StaticTask_t control;
  static_assert(kStackBytes % sizeof(StackType_t) == 0);
  alignas(8) static StackType_t stack[kStackBytes / sizeof(StackType_t)];
  static osThreadId_t thread = nullptr;
  if (thread || osKernelGetState() != osKernelRunning) return false;
  osThreadAttr_t attributes{};
  attributes.name = "Task_Imu";
  attributes.priority = osPriorityAboveNormal;
  attributes.cb_mem = &control;
  attributes.cb_size = sizeof(control);
  attributes.stack_mem = stack;
  attributes.stack_size = sizeof(stack);
  thread = osThreadNew(Task_Imu, nullptr, &attributes);
  return thread != nullptr;
}

bool ReadLatestImu(ImuReading& reading) {
  taskENTER_CRITICAL();
  reading = g_imu_latest;
  taskEXIT_CRITICAL();
  return reading.sample.valid;
}

}  // namespace app

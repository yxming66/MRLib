#pragma once

#include "device/imu/imu_sample.h"

namespace app {

struct ImuReading {
  // BMI088 传感器坐标。装入机器人后，使用前按实物安装方向转换到机体坐标。
  MrImuSample sample{};
  float temperature_c = 0.0f;
  uint32_t failed_frames = 0;
  int8_t init_result = -1;
};

// 在调度器运行、SPI/GPIO 登记和 DMA 策略安装之后调用一次。
bool CreateImuTask();
// 从任务上下文复制最新快照。返回值表示有有效采样；失败时仍可查看诊断字段。
bool ReadLatestImu(ImuReading& reading);

}  // namespace app

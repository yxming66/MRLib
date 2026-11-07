# 任务归项目所有

本目录的旧整机任务、配置和 `user_task.h` 已移除。库不再默认创建某台机器人的线程、队列、电源和全局对象。

`periodic_task.hpp` 提供显式选择的 CMSIS-RTOS2 周期任务外壳，用于项目的 FreeRTOS 内核。调用者提供 Init/Step/Fault 回调和配置；只有项目调用 Create 才会创建线程。任务在内核启动前创建，初始化和业务回调在任务内执行；不拥有机器人模块或板级资源。具体的 YAML → 配置 → 手写底盘业务例子见 [chassis_task](../../examples/chassis_task/README.md)。首版只有单个周期任务生成，不恢复旧的全局队列和整机任务集合。

提取出的可复用能力：

- `module/attitude/attitude.hpp`：接收 IMU 快照的姿态估计；`component/attitude/gyro_bias.hpp`：连续静止样本的零偏校准。
- `module/audio/alarm.hpp`：独立于 OS tick 的告警超时、最短保持和交替音调输出。
- `component/time/periodic.hpp`：显式时间驱动的周期到期判断；`component/timer.hpp`：注入单调时钟的计时器。
- 已有的 `device/buzzer`、`module/cloudmusic` 仍保存播放器/曲库；它们尚未完成板级与遥控菜单解耦。

项目在自己的 `app/` 中创建实例和周期任务，在 `board/` 中读取设备并下发输出。姿态估计前先统一传感器坐标轴，每个序列号只发布一次完整快照；校准输入应是未经旧零偏扣除的原始角速度。校准完成会将姿态重置为单位四元数。

旧任务中的固定校准值、轮速阈值、动态 Z 偏置、加热目标/PID、按钮等待/提示音、呼吸效果、拨杆映射、系统复位和全局 profiler 不作为默认库行为保留。需要时由项目显式选择；没有把废弃整机实现改名为可运行示例。

主机测试不代替 STM32 固件和实机验证。具体迁移状态见 [重构进度](../../docs/USER_REFACTOR_PROGRESS.md)。

# 任务归项目所有

本目录的旧整机任务、配置和 `user_task.h` 已移除。库不再默认创建某台机器人的线程、队列、电源和全局对象。

`periodic_task.hpp` 是可选的 CMSIS-RTOS2 周期执行器，项目直接提供配置、Init/Step/Fault 回调、静态栈与控制块。它不拥有模块或板级资源，不使用入口名称注册或 Python 生成连接。调用者在内核初始化后、启动前创建任务，业务在任务内执行。可复制修改的例子见 [chassis_task](../../examples/chassis_task/README.md)。任务也可以完全手写，不要求使用该外壳。

提取出的可复用能力：

- `module/attitude/attitude.hpp`：接收 IMU 快照的姿态估计；`component/attitude/gyro_bias.hpp`：连续静止样本的零偏校准。
- `module/audio/alarm.hpp`：独立于 OS tick 的告警超时、最短保持和交替音调输出。
- `component/time/periodic.hpp`：显式时间驱动的周期到期判断；`component/timer.hpp`：注入单调时钟的计时器。
- 已有的 `device/buzzer`、`module/cloudmusic` 仍保存播放器/曲库；它们尚未完成板级与遥控菜单解耦。

项目在自己的 `app/` 中创建实例和周期任务，在 `board/` 中读取设备并下发输出。姿态估计前先统一传感器坐标轴，每个序列号只发布一次完整快照；校准输入应是未经旧零偏扣除的原始角速度。校准完成会将姿态重置为单位四元数。

旧任务中的固定校准值、轮速阈值、动态 Z 偏置、加热目标/PID、按钮等待/提示音、呼吸效果、拨杆映射、系统复位和全局 profiler 不作为默认库行为保留。需要时由项目显式选择；没有把废弃整机实现改名为可运行示例。

主机测试不代替 STM32 固件和实机验证。具体迁移状态见 [重构进度](../../docs/USER_REFACTOR_PROGRESS.md)。

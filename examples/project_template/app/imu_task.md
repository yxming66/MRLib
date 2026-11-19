# H723 板载 BMI088 读取示例

`imu_task.cpp` 在独立 FreeRTOS 任务中初始化 BMI088，等待加速度计和陀螺仪的数据就绪中断，使用 SPI2 DMA 顺序读取两颗芯片，再发布一份六轴快照。加速度为 m/s²，角速度为 rad/s，温度为 °C。时间戳是本板完成整帧读取的微秒时刻。数据采用传感器坐标，接入姿态解算前须按实际安装方向变换到机体坐标。陀螺仪偏置可在 `kGyroCalibration` 填写，默认是零。

| 用途 | H723 工程接线 | MRLib 端点 |
| --- | --- | --- |
| IMU 总线 | SPI2 | `spi2` |
| 加速度计片选 | PC0 | `accel_cs` |
| 陀螺仪片选 | PC3 | `gyro_cs` |
| 加速度计数据就绪 | PE10 / EXTI15_10 | `accel_int` |
| 陀螺仪数据就绪 | PE12 / EXTI15_10 | `gyro_int` |

## 接入项目模板

1. 在 Cube 将 SPI2 配成双线全双工、8 位主机、软件 NSS。本 H723 工程采用 SPI 模式 3（高电平空闲、第二边沿采样）。SPI2 RX DMA 使用 Normal 模式，内存和外设数据宽度均为 Byte。启用 SPI2、DMA RX 和两个数据就绪引脚的中断，优先级须允许调用 FreeRTOS ISR API。
2. 两路片选默认置高。当前驱动把两路数据就绪配置为低有效，因此 Cube 中 PE10 和 PE12 都使用 `GPIO_MODE_IT_FALLING`。`EXTI15_10_IRQHandler` 须分别调用两个引脚的 `HAL_GPIO_EXTI_IRQHandler`。本 H723 工程原有 GPIO 配置是上升沿，复制接入时要在 `.ioc` 同步修改。
3. 按 [项目模板](../README.md) 完成时基和端点登记。启用 IMU 的板在 `MX_*_Init` 完成后调用 `Board_RegisterSpiEndpoints()` 和 `Board_RegisterGpioEndpoints()` 并检查返回值，然后安装板级 `BSP_DMA_Configure` 钩子。H723 示例工程将 `.dma_buffer` 放在 DMA1 可访问的 RAM_D2。沿用该工程的 `Board_ConfigureDma()` 时须满足其 D-cache 关闭前提。启用 D-cache 的板需提供缓存同步策略。
4. CMake 配置添加 `-DMR_EXAMPLE_ENABLE_IMU=ON`。模板会编译任务、BMI088 驱动及 SPI/GPIO/DMA BSP，并在 `Task_Init` 创建 IMU 线程。示例按 1 ms RTOS tick 设置等待时间。IMU 和 DR16 同时开启时共用同一份 DMA BSP。

## 验证采样

在调试器的变量窗口观察 `g_imu_latest`。`init_result == 0` 表示初始化和两颗芯片的 ID 检查通过，`sample.valid == true` 且 `sample.sequence` 持续增加才表示收到完整六轴帧。板静止时，加速度向量的模约为 9.8 m/s²，陀螺仪三个轴应接近零。翻转板子后，重力在各轴上的分量随之变化。

`init_result == -4` 表示芯片 ID 不符，先检查 SPI2 和片选。初始化成功但 `failed_frames` 持续增加时，核对两路 EXTI 边沿和 SPI2 DMA 完成中断，并确认 DMA 缓冲区地址被板级策略接受。采集失败会撤销 `sample.valid`，保留上一次成功帧的序号和时间戳。

其他任务用 `app::ReadLatestImu(reading)` 拷贝完整读数。对数据时效有要求的任务，还应以 `BSP_TIME_Get_us()` 检查 `sample.timestamp_us`。

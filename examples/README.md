# 使用示例

- [项目接入文件](project_template/README.md)：可复制的 app、板级时基及 FDCAN/UART/GPIO/SPI/PWM 登记文件。
- [云台周期任务](project_template/app/gimbal_task.md)：基础 yaw-pit 云台的 `GimbalTask`、IMU/编码器职责划分和项目接入示例。
- [CmdCenter 多输入示例](project_template/app/cmd_center.md)：独立 cpp、右拨杆模式回调、DR16/NUC 路由、仲裁与安全命令，含独立主机测试。
- [DR16 遥控器任务](project_template/app/remote_control_task.md)：阻塞式 DMA 接收和受保护的最新快照，供 CmdCenter 读取。
- [板载 BMI088 读取](project_template/app/imu_task.md) 使用 H723 SPI2、数据就绪中断和 DMA 获取六轴数据及温度。
- `mrlink/`：MRLink 通信示例。
- `music/`：音乐相关示例。

按功能选择文件，示例目录不参与库目标的自动编译。

项目模板已删除旧 app_channels 单槽队列；命令由所有者任务直接取得，状态通过 ReadChassisStatus 读取。迁移时不再调用 InitChannels/PublishChassisCommand。

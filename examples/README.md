# 使用示例

- [项目接入文件](project_template/README.md)：可复制的 app、板级时基及 FDCAN/UART/GPIO/SPI/PWM 登记文件。
- [云台周期任务](project_template/app/gimbal_task.md)：基础 yaw-pit 云台的 `GimbalTask`、IMU/编码器职责划分和项目接入示例。
- [DR16 遥控器任务](project_template/app/remote_control_task.md)：阻塞式 DMA 接收任务、通用遥控输入快照及底盘命令映射示例。
- `mrlink/`：MRLink 通信示例。
- `music/`：音乐相关示例。

按功能选择文件，示例目录不参与库目标的自动编译。

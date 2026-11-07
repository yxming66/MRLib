# BSP 板级映射核查

2026-11-07：当前 BSP 仍有板型限制，编译和链接成功不等于所有 BSP 已与原板型解耦。

| 部分 | 当前情况 | 后续边界 |
| --- | --- | --- |
| UART | 已删除 RC/PC 用途及固定句柄引用，board 登记、项目代码选择用于 H7 MRLink，DR16 显式注入端点 | 引脚/波特率/DMA/IRQ 属 Cube；最多十端点、单次启动登记 |
| FDCAN | 项目可选择 BSP_CAN_1/2/3，具体 hfdcan1/2/3、过滤与资源布局仍在 H723 BSP | 待迁移句柄及控制器参数到 board，不承诺任意板型 |
| SPI | BSP_SPI_BMI088 固定 hspi2/SPI2，BMI088 使用固定 SPI/CS | 待拆端点登记、设备注入及总线/CS 所有权 |
| GPIO | GPIO_Map 包含 IMU CS/IRQ、电源、按键和矿石/矛头继电器等用途 | 待移入 board 并整理重复项 |
| PWM | 已删除业务枚举和固定 htim，board 登记时钟/通道，设备显式选择 | 共享 timer 禁止单通道改频；当前支持 PWM1 向上计数、CH1..4；不支持波形 DMA |
| 时基 | time.c 依赖 H723 TIM2 + HAL tick | 其他板型需提供明确实现 |

UART/PWM 已完成板级登记拆分；FDCAN/SPI/GPIO/时基仍是已确认待办，不应归为“板级配置已完成”。外部 rm_balance/zzrobot/RC_02 未修改。

PWM 接入与迁移见 [PWM_BINDINGS.md](PWM_BINDINGS.md)。

# BSP 板级映射核查

2026-11-07：当前 BSP 仍有板型限制，编译和链接成功不等于所有 BSP 已与原板型解耦。

| 部分 | 当前情况 | 后续边界 |
| --- | --- | --- |
| UART | 已删除 RC/PC 用途及固定句柄引用，board 登记、项目代码选择用于 H7 MRLink，DR16 显式注入端点 | 引脚/波特率/DMA/IRQ 属 Cube；最多十端点、单次启动登记 |
| FDCAN | 已删除固定句柄/过滤宏，board 登记命名端点并维护过滤器和共享 RAM；app 通过 FindEndpoint 选择 | H7 HAL 后端、最多三路；board 保证 RAM 容量/所有权与 IRQ 优先级，任务串行操作 |
| SPI | 已删除固定 hspi2/SPI2 与用途枚举，board 登记总线；BMI088 显式接收 SPI/CS/IRQ | 8 位主机双线、无 CRC、单所有者；不提供共享总线仲裁 |
| GPIO | 已删除用途映射与重复错绑，board 登记管脚/角色；逐条 EXTI 掩码 | 当前面向 H7 M7，Cube 负责路由/边沿/NVIC；其他芯片需适配 |
| PWM | 已删除业务枚举和固定 htim，board 登记时钟/通道，设备显式选择 | 共享 timer 禁止单通道改频；当前支持 PWM1 向上计数、CH1..4；不支持波形 DMA |
| 时基 | time.c 直接链接项目 Board_Time*；示例 board 复用 TIM2，独立 64 位累计 | H7 示例 TIM2 优先级 0、IRQ 延迟小于 1 ms；不支持停 tick/运行期换时钟/低功耗连续计时 |

UART/PWM/SPI/GPIO/FDCAN 的硬件选择及时间源已拆到 board；这不等于当前 BSP 可以直接用于任意 MCU，也不代表已通过板上验收。外部 rm_balance/zzrobot/RC_02 未修改。

FDCAN 接入与迁移见 [FDCAN_BINDINGS.md](FDCAN_BINDINGS.md)，时间源合同见 [TIME_SOURCE.md](TIME_SOURCE.md)。

PWM 接入与迁移见 [PWM_BINDINGS.md](PWM_BINDINGS.md)。

GPIO/SPI 与 BMI088 接入见 [GPIO_SPI_BINDINGS.md](GPIO_SPI_BINDINGS.md)。

# UART 板级端点与用途分配

UART 驱动不编码 RC/PC/IR 用途，不引用 Cube 的 huart 全局变量或固定 UART 实例。board 登记硬件，app 通过端点访问设备。不需要 YAML。

## board 登记

Cube 初始化完成后、启动设备和任务前调用一次：

```c
#include "bsp/uart.h"
#include "usart.h"
static const BSP_UART_Endpoint_t endpoints[] = {
    {"usart1", &huart1}, {"uart7", &huart7}, {"uart9", &huart9},
};
if (BSP_UART_Configure(endpoints, 3) != BSP_OK) Error_Handler();
```

表复制到驱动；名称和 HAL 句柄必须具有静态生命周期。空表、超过十端点、空名称/句柄、未就绪句柄、重复名称或物理外设会被拒绝。整表校验后发布，失败无部分映射，成功后禁止重配。

BSP_UART_0..9 是中性槽位，不代表硬件编号或用途。HAL 回调按已登记的准确句柄分发；即使 Instance 相同，未登记的另一个句柄也不会接收回调。未登记端点的占用、回调注册和收发失败。

## 项目选择用途

```cpp
// board/bindings.hpp
inline constexpr const char* kTelemetryUart = "usart1";
// app initialization, after board registration
const auto uart = BSP_UART_FindEndpoint(kTelemetryUart);
if (uart == BSP_UART_ERR) { /* report initialization failure */ }
// Check telemetry.Begin(uart) / DR16_Init(&receiver, uart) results.
```

H7 实际选择位于 board/bindings.hpp，将 usart1 改为 uart9 并构建即可切换遥测句柄和回调，前提是 UART9 已初始化并登记。引脚、波特率、DMA、IRQ、Cache 由 Cube/board 管理。

## 迁移

旧 BSP_UART_PC/RC/IR 等枚举已删除；使用 FindEndpoint。DR16_Init 增加显式串口参数，当前仍只支持一个接收机。旧 YAML board_bindings.hpp 与 CommunicationUart 生成查找已删除，项目用普通 C++ 常量表达用途，不维护第二份硬件能力清单。

驱动测试覆盖十槽位生命周期、注册冲突和 DR16 端点注入；完整 H7 示例验证链接。本轮未进行真实波形、DMA/IRQ 或板上验证。

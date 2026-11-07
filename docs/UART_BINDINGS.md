# UART 板级端点与 YAML 绑定

库已删除 RC/PC/IR 等用途枚举；uart.c 不再引用 Cube 的 huart 全局变量或具体 UART 实例。通用驱动、board 硬件登记与项目用途分开维护。

## 项目分配

```yaml
board:
  bindings:
    uart:
      telemetry_uart: usart1
      remote: uart9
```

值是 board 登记的硬件名，同一端点不能赋给两个逻辑名称。改成 telemetry_uart: uart7 后重新构建就更新绑定，前提是 board 初始化、登记 UART7，CMake 也声明它可用。引脚、波特率、DMA、IRQ 和 Cache 仍由 Cube/board 配置。

生成的 board_bindings.hpp 提供：

```cpp
mr::generated::Uart("remote");                 // 逻辑名称 → BSP 端点
mr::generated::CommunicationUart("telemetry"); // communication → port → BSP 端点
mr::generated::ValidateUartBindings();        // 检查实际硬件登记
```

查找失败返回 BSP_UART_ERR。在 board 登记后调用，不在全局构造阶段缓存结果。H7 app 已通过 CommunicationUart 连接真实 MRLink；改 YAML 会改变 HAL 句柄及回调分发。通信缓冲、codec、轮询和任务仍手写，尚未生成完整服务。

BSP_UART_0..9 是最多十个中性槽位，不等于 UART0..9，不表示用途。设备应查名字，不猜槽位。

## board 登记

Cube 初始化完成后、启动设备接收和任务前，调用一次：

```c
#include "bsp/uart.h"
#include "usart.h"  /* Cube 头只由 board 引入 */
static const BSP_UART_Endpoint_t endpoints[] = {
    {"usart1", &huart1}, {"uart7", &huart7}, {"uart9", &huart9},
};
if (BSP_UART_Configure(endpoints, 3) != BSP_OK) Error_Handler();
```

表会被复制，名称和句柄须保持运行期有效。配置拒绝空表、超过十端点、空名称/句柄、未就绪句柄、重复名称及同一物理外设的重复句柄；失败不发布部分映射，成功后禁止重配。不能在 DMA 活跃时改变映射。未登记端点的回调注册、占用和收发均失败，不回退到旧分配。

HAL 回调按登记句柄找槽位，未知句柄即使 Instance 相同也不转发。已有 DMA 所有权/释放规则保持适用；board 不应重复定义 BSP 提供的 HAL 回调，Cube IRQ 仍调用 HAL handler。

实际表见 examples/h7_integration/board/uart_endpoints.c，只列硬件、不分用途。CMake 同时如实声明能力：

```cmake
mr_register_board_adapter(TARGET board_adapter PROFILE my_board
    CAN_ENDPOINTS BSP_CAN_1 UART_ENDPOINTS usart1 uart7 uart9)
```

当前 H723 帮助函数也接受 UART_ENDPOINTS。声明不代替实际登记：构建时检查能力，启动时 ValidateUartBindings 检查实际表。新增硬件只更新 Cube/board 和能力清单，无需改库枚举。

## 旧项目迁移

本批更改了 BSP/DR16 的 C API，升级库需同步迁移：

1. board 显式登记硬件句柄。
2. YAML 的 telemetry_uart: BSP_UART_PC 改为 usart1 等实际名称。v2 字段结构和名称字符串契约不变，H7 board 不再提供旧用途名。
3. 手写的 BSP_UART_PC/RC/IR 等改为 FindEndpoint 或生成的逻辑查找；旧枚举已删除，没有隐式兼容映射。
4. DR16_Init(&receiver) 改成 `DR16_Init(&receiver, mr::generated::Uart("remote"))`；无 YAML 可传 FindEndpoint 的结果。当前仍只支持一个 DR16 实例。

生成头独立于 robot_config.hpp，可移植模块不必引入 HAL。无 YAML 的项目仍可直接登记、查找和手写装配。

回归覆盖真实 UART 源、YAML 改端点后的 CMake 重建、十槽位 MRLink 生命周期和 DR16 改用 UART9。未进行波形、DMA/IRQ 或板上验证。

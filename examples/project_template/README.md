# 可复制的项目接入文件（H723 / FreeRTOS）

这是普通 C/C++ 示例文件，复制后归项目维护，不涉及生成器。以 engineering_hero 的底盘板为基线；不是完整 Cube 工程，不提供启动文件、HAL 和链接脚本。先用自己的 .ioc 生成并编译通过 Cube 工程。

## 文件选择与修改位置

| 文件 | 用途 | 新项目要改什么 |
| --- | --- | --- |
| board/fdcan_endpoints.c/.h | CAN 句柄、过滤器、共享 RAM 检查 | 实际启用的控制器、FIFO、过滤器数量和 RAM 规划 |
| board/time_source.c/.h | 微秒时间源及更新钩子 | 定时器、APB 时钟与 Cube HAL 时基；本例为 TIM23 |
| board/uart_endpoints.c/.h | UART 命名登记 | huart 句柄与名称，波特率在 Cube 改 |
| board/gpio_endpoints.c/.h | GPIO/EXTI 命名登记 | main.h 引脚宏和输入/输出/EXTI 模式 |
| board/spi_endpoints.c/.h | SPI 命名登记 | 总线句柄，Cube 配置须满足 BSP_SPI 契约 |
| board/pwm_endpoints.c/.h | PWM 命名登记 | 定时器、通道、真实内核时钟、位宽与共享关系 |
| board/bindings.hpp | 项目功能使用哪个物理端点 | 名称必须匹配对应登记表 |
| app/robot_config.hpp | 几何、ID、周期、超时、方向等参数 | 按实际机器人填写；示例值不是通用默认值 |
| app/chassis_assembly.hpp | 静态创建电机、轮组和底盘对象 | 电机型号、轮数、控制配置 |
| app/chassis_task.cpp/.hpp | 创建静态 FreeRTOS 任务并循环运行 | 任务周期、优先级和业务 |
| app/app_messages.hpp | 本板任务通信数据结构 | 命令和反馈内容 |
| app/app_channels.cpp/.hpp | 静态单槽队列 | 增加消息通道，命令单消费者，状态可多读者 |
| app/app_init.cpp/.h | 初始化线程入口与创建业务任务 | 本板要启动的任务 |
| cmake/mr_sources.cmake | 显式源码和库 target 清单 | 相对库路径、选用的外设和功能 |

每个板子分别拥有 board、app、Cube 工程和构建目标，公共库在机器人仓库 libraries 中保留一份。只复制需要的文件；UART/GPIO/SPI/PWM 是独立可选项。

## 复制与构建接入

建议结构：

```text
robot_project/
  libraries/MRLib/
  firmware/chassis_board/
    Core/ Drivers/ Middlewares/     # Cube 生成
    app/                           # 复制本例 app
    board/                         # 复制时基、FDCAN、bindings；其他按需
    cmake/mr_sources.cmake          # 复制本例 cmake 文件
    CMakeLists.txt
```

本板 CMakeLists.txt 启用 CXX、C++17，在 add_subdirectory(cmake/stm32cubemx) 后加入：

```cmake
include(cmake/mr_sources.cmake)
```

这段清单用于新工程。已有工程已手写相同目标时，选一种写法保留，避免重复定义目标、重复编译 BSP。不要递归扫描编译 examples 或整个 User。Cube 生成的链接语句通常是 plain 写法，本例保持一致。

## main.c 需要添加的内容

在 USER CODE Includes 中引入 board/time_source.h、board/fdcan_endpoints.h。可选外设也引入对应 board/*_endpoints.h。

在 SystemClock_Config 与 SystemPower_Config 之后、使用库计时之前（本例 USER CODE SysInit）：

```c
if (!Board_TimeInit()) Error_Handler();
```

在全部 MX_*_Init 完成后、osKernelInitialize 之前（USER CODE 2）：

```c
if (!Board_RegisterFdcanEndpoints()) Error_Handler();
// 选用哪个外设，才编译对应文件并取消对应调用注释：
// if (!Board_RegisterUartEndpoints()) Error_Handler();
// if (!Board_RegisterGpioEndpoints()) Error_Handler();
// if (!Board_RegisterSpiEndpoints()) Error_Handler();
// if (!Board_RegisterPwmEndpoints()) Error_Handler();
```

在现有 HAL_TIM_PeriodElapsedCallback 的 USER CODE Callback 0 加入：

```c
Board_TimeOnUpdate(htim);
```

保留 Cube 原有对 HAL_IncTick() 的调用且只能调用一次。不要再定义第二个同名 HAL 回调。TIM23 是本例 Cube 已启动的 HAL 时基，1 MHz 计数、ARR=999、APB2 /2；其他板子必须调整源码的校验和计时实现。更新中断优先级为 0，不能调用 FreeRTOS API；最大屏蔽/延迟应小于 1 ms，不适用于暂停该定时器的低功耗流程。

## 任务如何启动

在 Cube freertos.c 的 USER CODE Includes 引入 app/app_init.h，在 MX_FREERTOS_Init 的 USER CODE RTOS_THREADS 中加入：

```c
if (!App_CreateInitTask()) Error_Handler();
```

执行顺序：main 完成 HAL/时基/端点登记 → osKernelInitialize → MX_FREERTOS_Init 创建 Task_Init → osKernelStart → Task_Init 建消息通道 → BSP_FDCAN_Init 启动已登记 CAN → 创建 Task_Chassis → 初始化线程退出 → 底盘任务每 1 ms 读取反馈和命令、发布状态。

App_CreateInitTask 只调用一次；BSP_FDCAN_Init 必须等 RTOS 正在运行后调用。其他设备的回调和收发启动放在初始化线程或其所属任务中，启动后不要重新登记端点。

## UART、GPIO、SPI、PWM 怎么用

登记表只连接硬件和名称，不会自动开始串口收发、启动 PWM 或改变电源输出。

```cpp
#include "board/bindings.hpp"
#include "bsp/uart.h"
#include "bsp/gpio.h"
#include "bsp/pwm.h"

// 设备初始化时取得端点，先检查端点是否有效。
auto uart = BSP_UART_FindEndpoint(board::bindings::kTelemetryUart);
auto cs = BSP_GPIO_FindEndpoint(board::bindings::kAccelCs);
auto buzzer = BSP_PWM_FindEndpoint(board::bindings::kBuzzerPwm);
```

UART：登记后安装回调，再调用 BSP_UART_ReceiveToIdle 或 Transmit。false 表示中断异步模式，true 表示 DMA；缓冲区必须一直有效到完成回调。设备驱动使用 Claim 和 Owned 回调避免抢占串口。BSP 提供 HAL UART 回调入口，不要在项目中重复定义。推荐 ReceiveToIdle + RxEvent 回调，保留 Cube 的 HAL_UART_IRQHandler 即可，不调用 BSP_UART_IRQHandler（它会提前清除 IDLE 标志）。只有使用旧 IDLE_LINE 回调模式时才添加该 IRQ 钩子，并自行启用 IDLE 中断；同一串口不能混用两种 IDLE 处理方式。

GPIO：BSP_GPIO_WritePin 设置输出；EXTI 必须先 RegisterCallback 再 EnableIRQ。边沿、路由、共享 NVIC 和 HAL_GPIO_EXTI_IRQHandler 仍由 Cube 生成和配置，回调在中断执行。

SPI：本例只登记 SPI2，支持 8 位主机、双线、无 CRC。一个总线由一个任务管理，或项目自己加锁；片选及连续传输由项目/设备管理。SPI 的异步缓冲区同样要保持有效到完成。

PWM：显式 SetComp/SetPulseUs 后才 Start。共享定时器的多个通道使用相同频率，本例全部禁止独立改频，避免影响其他通道；专用定时器才可在登记表允许 SetFreq。

每次调用都检查 BSP_OK/错误返回值。UART/SPI 要同时编译 User/bsp/dma.c，即使先只用中断模式。H7 使用 DMA 前还必须由 board 安装 BSP_DMA_Configure 的校验/缓存同步钩子；没有钩子会拒绝 DMA。钩子须确认 DMA 可访问内存（不能直接使用 DTCM）、对齐和 D-cache 一致性，见 User/bsp/dma.h 的契约。本例未提供通用 DMA 内存策略，首次接入可先使用中断模式。

## 当前底盘示例的范围

麦克纳姆、4 个 M3508、0x201～0x204 顺序为右前/左前/左后/右后；轮径 0.18 m、前后中心距 0.60 m、左右中心距 0.50 m，额外减速比 1。M3508 自带减速比已在库中处理。

示例只采集反馈、处理命令通道和发布状态，control_enabled 保持 false，不发送电机控制帧。方向确认、PID/限幅、命令来源、故障处理和控制输出仍须由项目完成；不能复制后直接视为可行驶底盘。CAN 表基于当前 .ioc：Cube 初始化 FDCAN1/2/3（均参与 RAM 检查），库登记 FDCAN1 与 FDCAN3。换板或仅开一路时须同步精简句柄、检查和过滤器。

新增模块时，增加该模块的装配/任务/消息和 CMake 依赖即可，不必新增模板生成器。维护者仅在公共接口变化时同步更新这些示例；复制出去的项目按自己的版本维护，不自动覆盖。

## 本次验证

使用 engineering_hero 的 H723 HAL/FreeRTOS 头文件和 ARM GCC，对本例全部 9 个 C/C++ 实现及 UART/GPIO/SPI/PWM/DMA 共 5 个 BSP 实现进行了严格编译（-Wall -Wextra -Wpedantic -Werror），全部通过。现有底盘固件 Debug 编译与链接通过。未执行烧录或板上验证；可选外设的运行行为仍需在实际板上验收。

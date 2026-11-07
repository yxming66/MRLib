# 板级接入与手写装配

准备能编译的 Cube/CMake 工程，保持其启动汇编、main、链接脚本、HAL/RTOS、引脚时钟、DMA/Cache 和中断。引入整个库，按 target 选择编译能力，无需复制库文件或运行生成器。

```cmake
project(my_robot LANGUAGES C CXX ASM)
add_subdirectory(external/MR_TestLib)
add_executable(robot_firmware board/main.c app/robot.cpp)
target_link_libraries(robot_firmware PRIVATE
    cube_platform board_adapter
    mr::module_chassis mr::device_motor mr::device_wheel
    mr::module_arm mr::device_joint)
```

cube_platform/board_adapter 由项目提供实际源文件、HAL include、芯片宏及 BSP 实现。链接哪些模块和设备，应与 app 实际使用一致。公共依赖由库的 CMake target 传递。

## 项目文件职责

- board/bindings.hpp：选择实际 CAN、UART 等资源。UART/FDCAN 在 Cube 初始化后调用 BSP_UART_Configure/BSP_FDCAN_Configure 登记，再通过各自 FindEndpoint 查找，不在全局构造阶段缓存查找结果。
- app/robot_params.hpp：单位明确的 PID、几何、方向、传动比、零点、限位、控制周期等。库 Config 类型做初始化校验。
- app/*_assembly.hpp/.cpp：普通 C++ 对象创建与连接，不需要公共装配基类或运行期注册系统。机械臂固定 Arm<N>，设备静态持有。
- app/tasks：手写或复制任务示例，直接调用 Init/Step/Fault 或反馈/控制/提交接口。不同项目自由选择队列、通知、调度与故障策略。

完整例子见 [H7 工程](../examples/h7_integration/README.md)，较小的任务例子见 [底盘任务](../examples/chassis_task/README.md)。复制示例后修改的文件完全归项目所有，没有重新生成或覆盖流程。

## 当前 H723 BSP 帮助函数

```cmake
include(external/MR_TestLib/cmake/MrStm32H7.cmake)
mr_add_stm32h7_bsp(TARGET board_adapter PLATFORM cube_platform
    PROFILE h723_zzrobot COMPONENTS can mrlink pwm gpio spi)
target_link_libraries(robot_firmware PRIVATE board_adapter)
```

这是现有 H723 实现的源文件选择函数，不是通用板卡注册系统：can 选择 fdcan/time/mm；uart、spi 分别携带 dma；mrlink 选择四个通道及 can/uart 依赖。pwm、gpio 仅选择对应实现，不引入 DMA/RTOS 依赖。它不生成代码、扫描 Cube、初始化硬件或分配用途。其他外设显式接入，换板按实际情况替换 BSP 实现。

启动顺序：HAL/最终时钟 → board 时间源就绪 → 其余外设 → UART/PWM/GPIO/SPI/FDCAN 登记与过滤器配置 → 内核初始化与应用任务创建 → 内核启动 → DMA 策略/BSP → 设备与模块注册 → 有效反馈 → 显式使能。示例线程静态提供栈与控制块。BSP 提供的 HAL 回调不能与项目重复定义，Cube IRQ 仍调用 HAL handler。

当前其余 BSP 的板型限制见 [核查表](BSP_PORTABILITY_AUDIT.md)。FDCAN 的位时序、消息 RAM、硬件过滤器和 FIFO 路由归 board，RTOS 运行后在任务中调用 BSP_FDCAN_Init；时间由项目直接实现 Board_Time*。见 [FDCAN 接入](FDCAN_BINDINGS.md) 与 [时间源](TIME_SOURCE.md)。生成配置链路已移除；如果从旧版本迁移，改用 app 参数与手写装配、显式链接库 target，删除项目对 MrRobot.cmake/生成头的依赖，并使用新的空 build 目录。

PWM 端点在 Cube 初始化后、输出启动前登记，实际定时器时钟由 board 提供。共享 timer 的周期统一归 board 设置，业务只能分别改脉宽。见 [PWM 接入](PWM_BINDINGS.md)。

GPIO/SPI 硬件表与 BMI088 参数接入见 [GPIO/SPI 接入](GPIO_SPI_BINDINGS.md)。登记后 EXTI 线保持屏蔽，安装回调后逐线使能，不关闭共享 NVIC 组。

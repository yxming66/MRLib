# 板级接入与生成底盘

库接入已有 Cube/CMake 工程。项目保留芯片启动汇编、链接脚本、HAL/RTOS、引脚时钟、DMA/Cache 策略、中断以及 main。生成器只提供设备/模块静态装配和库依赖，不生成第二套硬件初始化。

## 在已有工程接入

准备 C/C++17 固件 target、平台头文件/宏 target 和 board 适配 target 后：

```cmake
project(my_robot LANGUAGES C CXX ASM)
add_subdirectory(external/MR_TestLib)
include(external/MR_TestLib/cmake/MrRobot.cmake)

# board_adapter 必须已实现实际使用的 BSP/CanPort，并传递 HAL 头文件和宏。
# profile 与 config.yaml 的 board.profile 相同，端点列表只声明已实现的能力。
mr_register_board_adapter(TARGET board_adapter PROFILE my_board
    CAN_ENDPOINTS BSP_CAN_1)
mr_target_robot_config(TARGET robot_firmware
    CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
    MODULE chassis BOARD_TARGET board_adapter
    PYTHON "${CONFIG_PYTHON}")
```

`CONFIG_PYTHON` 为安装了 `tools/requirements-config.txt` 的 Python 3.10+ 解释器。省略 PYTHON 时 CMake 查找 Python3。`robot_firmware` 与 `board_adapter` 必须先存在；固件继续显式链接 Cube/HAL/RTOS。配置文件的 `project.library` 指向此库 checkout。

配置阶段校验并生成 `${CMAKE_CURRENT_BINARY_DIR}/generated`，核对 board profile、CAN 端点，再把生成头文件路径、`MR_ROBOT_CAN_BINDING` 宏及库 target 加入固件。YAML 和生成器/Schema 的改变会触发 CMake 重新配置。可以用 OUTPUT_DIRECTORY 改为另一个专属 generated 目录，每个固件配置使用独立目录。

当前支持选定的单总线 RM 四轮底盘和 DM/LZ 串联机械臂。将生成的 `mr::generated::ChassisAssembly` 作为静态对象，注入该总线的 CanPort，调用 Register；app 继续负责反馈、命令、Step、CommitOutput 与 RM 分组 Flush。它不创建任务或自动使能；周期常量供 app 调度使用。生成的 `ArmAssembly` 持有电机 → MotorActuatorAdapter → JointT → Arm<N>，注入同一 CanPort 后调用 Init，再由 app 继续 UpdateFeedback、显式 Enable、Step 和 CommitOutput。N 按关节列表长度推导，零点仅在 JointT 处理，模型 position_offset 保持零。完整调用顺序参考 H7 app 和 `examples/device_execution`。

## 可选的单周期任务

在 `mr_target_robot_config()` 中额外传入 `TASK chassis`，等价于 CLI 的 `--task chassis`。生成器输出任务配置与创建连接并自动将 `robot_tasks.cpp` 加入固件编译；board 提供 CMSIS-RTOS2/FreeRTOS 头文件和实现。app 必须实现入口绑定及 Init/Step/Fault，board 在内核初始化后、启动前调用生成的 `MR_CreateConfiguredTask()` 并检查结果。此时不要重复创建同一个手写任务。

首版只支持一个恰好拥有所选模块、services 为空的周期任务；动态分配需要 board 提供足够 heap。业务初始化仍在任务内运行，未生成多任务资源就绪屏障、通信或队列。具体文件、命令和边界见 [chassis_task](../examples/chassis_task/README.md)。默认 H7 整机示例继续使用原来的手写 RobotTask。

## 当前 H723 BSP 的可选帮助函数

当前 User/bsp 的 HAL 句柄与端点映射属于既有 H723/zzrobot 板型，可用：

```cmake
include(external/MR_TestLib/cmake/MrStm32H7.cmake)
mr_add_stm32h7_bsp(TARGET board_adapter PLATFORM cube_platform
    PROFILE h723_zzrobot COMPONENTS can mrlink)
```

`cube_platform` 提供 Cube 的 include、编译定义和所需平台使用条件，不应包含另一个 main。帮助函数创建 OBJECT target 并选择以下 MR 源依赖：

| 选择 | 自动加入 |
| --- | --- |
| can | fdcan.c、time.c、mm.c |
| uart | uart.c、dma.c |
| mrlink | 四个通道实现、can/uart 及其依赖、component_mrlink |

其他外设仍显式接入。函数不会编译 Cube、不配置硬件、不读取 YAML 的 cube_root；固定 profile 不能冒充任意 H7 板型。更换 PCB/句柄映射需提供新的 board 适配，而非只改 profile 名。

启动顺序由项目维持：HAL/时钟和 MX 外设初始化 → RTOS/任务 → DMA 策略与 BSP 初始化 → 设备/模块注册 → 反馈和显式控制。当前 BSP 的完成回调由库实现，不能再加入相同的 HAL 回调定义；Cube IRQ 仍调用 HAL handler。涉及 DMA 时 board 必须安装可达内存与 Cache 策略。现有 H7 示例关闭 Cache，不能据此推断开启 Cache 后也正确。

完整可编译工程、具体时基和内存前提见 [H7 示例](../examples/h7_integration/README.md)，参数字段和文件保护规则见 [配置规范](ROBOT_CONFIG.md)。

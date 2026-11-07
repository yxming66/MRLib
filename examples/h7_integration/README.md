# H723 独立固件集成示例

这个工程把当前库的真实代码接成完整 ELF/BIN：四个 RM M3508 → MotorWheel → 统一 Chassis，以及三个 DM J4310 → MotorActuatorAdapter → JointT → Arm<3>；MRLink 使用 USART1 DMA；一个 CMSIS-RTOS2 任务按 1 ms 调度控制、100 ms 发布遥测。

这是部分生成装配基线，用来验证接口、依赖、启动文件、HAL/RTOS 和链接布局。RM 四轮底盘及控制参数从 YAML 生成；机械臂、通信和任务创建仍手写。没有移植 zzrobot 的遥控策略、机械标定或整机业务。

## 文件与依赖

```text
h7_integration/
├── config.yaml              # v2 参数范本（生成 RM 底盘装配及参数）
├── CMakeLists.txt            # 选择库 target，显式装配 Cube/BSP 实现
├── arm-none-eabi.cmake       # Cortex-M7 / hard float / C11 / C++17
├── board/
│   ├── main.c               # 应用入口、板级时钟和 RTOS 启动
│   └── stm32h723.ld          # Flash/AXI SRAM、构造函数、heap/stack
└── app/
    ├── robot.cpp            # 静态设备所有权、模块配置、控制循环、遥测
    └── motor_config.hpp     # 可在主机上测试的手写电机装配参数
```

`board/main.c` 是应用入口；真正的向量表和复位入口由 Cube 的 `startup_stm32h723xx.s` 提供。默认生成产物在仓库 `build/h7-integration/generated/`。

需安装 CMake >= 3.22、GNU Arm Embedded 工具链，以及 Ninja 或 Make。`MR_CUBE_ROOT` 指向完整 STM32H723 Cube 工程，当前验证的是 `D:/STM32Projects_HAL/board-H7/zzrobot`。只读取其 `Core`、`Drivers`、`Middlewares` 和 `startup_stm32h723xx.s`，不编译旧 `main.c`、`freertos.c`、`User`，不修改外部文件。

板级输入并非任意 STM32 Cube 工程：要求 H723XG、24 MHz HSE、相同的 GPIO/SPI2/TIM/UART/FDCAN 句柄及初始化函数、TIM2 的 1 MHz/999 HAL 时基、1000 Hz FreeRTOS/CMSIS-RTOS2，以及相同引脚配置。换板必须替换 board 和 Cube 配置。

## 构建

从 MR_TestLib 根目录执行。PowerShell 的 `-D...` 参数整体加引号，避免路径、扩展名被重新拆分：

```powershell
py -3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r tools/requirements-config.txt
$configPython = (Resolve-Path .venv/Scripts/python.exe).Path
cmake -S examples/h7_integration -B build/h7-integration -G "MinGW Makefiles" `
  "-DMR_CONFIG_PYTHON=$configPython" `
  "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe" `
  "-DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake" `
  "-DMR_CUBE_ROOT=D:/STM32Projects_HAL/board-H7/zzrobot" `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/h7-integration --parallel 4
arm-none-eabi-nm -u build/h7-integration/mr_h7_integration.elf
arm-none-eabi-size build/h7-integration/mr_h7_integration.elf
```

其他系统可以使用 `-G Ninja`，省略 `CMAKE_MAKE_PROGRAM`。`arm-none-eabi-gcc/g++/objcopy/size` 必须在 PATH 中。构建产物在 `build/h7-integration/`：`mr_h7_integration.elf`、`.bin`、`.map`。`nm -u` 应没有输出。命令只构建，不烧录。

默认构建还会严格编译全部 35 个 User 生产 `.c/.cpp` 文件；`user_all_compile_check` 不链接进固件，以免未选功能产生重复定义。这能发现可选驱动的 include/语法问题，不能验证所有模板实例和所有功能的链接依赖。

公共头文件单独验证：

```powershell
py -3 tools/check_user_headers_h7.py --board-root D:/STM32Projects_HAL/board-H7/zzrobot
```

`.hpp` 按 C++17 检查，`.h` 额外按 C11 检查，不额外加入 `User/device` 等搜索路径掩盖缺失 include。

## 内存和运行约定

- Flash 为 `0x08000000` 起 1 MiB；数据、BSS、FreeRTOS heap、线程栈、MSP 都在 `0x24000000` 起 320 KiB 的 AXI SRAM，DMA1/2 缓冲不放 DTCM。启动代码复制 data、清零 BSS 并运行 C++ 构造函数。
- D-cache/I-cache 显式关闭；DMA hook 只接受上述 AXI SRAM。当前不是 Cache 开启后的性能模板。开启 Cache 时需独占缓存行、MPU/Cache 策略及实机一致性验证。
- FreeRTOS heap 72 KiB，robot 线程栈 32 KiB；链接额外预留 newlib heap 8 KiB、MSP 16 KiB。后续应实测任务栈高水位和最坏周期耗时。
- BSP 启动、设备注册与控制全部在同一 robot 任务中完成；CanPort、RM 分组总线、执行器、关节、模块和 MRLink 对象均为静态生命周期。UART/FD CAN IRQ 由 Cube 文件提供，HAL 完成回调由本库接管。
- 默认 YAML `runtime.allow_motor_tx: false`，MotorPort 拒绝所有电机 TX；`request_run=false`，不会自动 Enable。FDCAN 接收和串口遥测仍会启动。
- 允许电机发送的独立构建使用另一份 config.yaml，将 `runtime.allow_motor_tx` 改为 true，用 `-DMR_ROBOT_CONFIG=配置绝对路径` 指定，并使用另一个 build 目录。复制 YAML 后须调整 project.library 指向本库。旧 `MR_H7_ALLOW_MOTOR_TX` CMake 开关已移除。仍须人工请求 `request_run` 的上升沿才能尝试启动；串口没有使能命令。启动前要求真实轮/关节反馈，运行中故障停止并锁住，需撤销请求后重新请求。例子只发零底盘速度和机械臂保持命令，停止路径只重试零/禁用命令。

示例 CAN 地址、轮径、底盘几何、关节范围、PID、DM MIT 范围均是演示值。DM 配置范围必须与电机固件一致；内置减速比已经计入厂家协议物理量，只有额外外部机构传动才填 external_ratio。机械臂是无重力补偿的示意串联模型，不能视为 zzrobot 标定。

## 已验证与待验收

2026-11-07：GNU Arm 15.2.1 完成两种发送开关的编译和链接；默认构建检查 35 个实际实现，133 个公共头可独立编译；29 项主机 CTest 和 6 项 CLI 回归通过。ELF 保留 RobotTask、底盘控制、ArmController、逆运动学、真实 HAL/RTOS 和 BSP 分配符号，未发现未定义符号。

尚未烧录或上电验证。需在板上确认时钟/中断优先级、FDCAN 波特率和消息 RAM、UART DMA 接收恢复、传感器数据、单位/方向/限位、失联停机、任务周期/栈及物理急停，再作为整机迁移基础。机械臂/通信/任务生成、任意硬件资源绑定仍待实现。


## 统一参数格式

新增 [config.yaml](config.yaml) 作为其他机器人项目的 v2 配置范本，包含 board 绑定、电机/轮子/关节、统一模块、MRLink 和任务参数。详细单位、引用关系、校验及扩展规则见 [配置规范](../../docs/ROBOT_CONFIG.md)。它已驱动底盘静态装配与参数，修改后运行 `cmake --build build/h7-integration` 会自动重新配置并编译。底盘周期也用于当前共用控制循环的机械臂 dt；通信周期、任务栈/优先级、机械臂模型和 DM 参数仍手写。示例只接入名为 chassis 的模块，变更名称需同步修改 CMake 的 MODULE 参数。板级边界与依赖选择见 [板级接入](../../docs/BOARD_INTEGRATION.md)。

整理参数时修正了本示例的 DM/RM 地址重叠：三个 DM 控制 ID 改为 `0x11/0x12/0x13`，反馈为 `0x321/0x322/0x323`，避免 DM 保留地址 `can_id+0x200` 占用 RM 反馈 `0x201..0x204`。新增主机注册回归，使用实际手写装配参数验证四个 RM 与三个 DM 可以同时注册；单纯完成固件链接无法发现这种运行期配置冲突。

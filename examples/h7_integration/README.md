# H723 手写固件集成示例

完整的普通 C/C++ 项目：四个 RM M3508 → MotorWheel → 统一 Chassis，三个 DM J4310 → MotorActuatorAdapter → JointT → Arm<3>，以及 USART1 DMA MRLink。一个手写任务按 1 ms 控制、100 ms 遥测。设备、关节和机械臂固定存储，没有运行期关节工厂。

本示例用于验证接口、依赖、HAL/RTOS、启动和链接布局，不包含 zzrobot 的遥控策略、机械标定或整机业务。默认禁止电机发送，运行请求初始关闭。

## 文件职责

```text
h7_integration/
├─ CMakeLists.txt            # 显式链接库与 Cube/BSP，无 Python
├─ arm-none-eabi.cmake       # Cortex-M7 工具链
├─ board/
│  ├─ main.c                # 时钟、Cube 初始化、静态线程创建
│  ├─ bindings.hpp          # CAN/UART/PWM/GPIO/SPI 用途
│  ├─ uart_endpoints.c      # 已初始化的真实 UART 句柄清单
│  ├─ pwm_endpoints.c       # PWM 通道、时钟及共享 timer 约束
│  ├─ io_endpoints.c        # GPIO/SPI 硬件表与 IMU 中断下降沿
│  └─ stm32h723.ld           # 链接布局
└─ app/
   ├─ robot_params.hpp      # 几何、PID、限位、零点、周期、发送许可
   ├─ motor_config.hpp      # 实际使用的电机配置函数
   ├─ chassis_assembly.hpp  # 四电机/轮子/底盘对象连接
   ├─ arm_assembly.hpp      # 三电机/执行器/关节/Arm<3> 连接
   └─ robot.cpp             # 状态机、故障与控制循环、遥测
```

这些文件直接编辑、编译，没有 generated 目录或 YAML 输入。示例可作为项目起点，复制后由项目维护。底盘任务的小型版本见 [chassis_task](../chassis_task/README.md)，通过 MR_CHASSIS_TASK_EXAMPLE=ON 选择。

board/main.c 是应用入口，不是芯片启动汇编。向量表/复位入口使用 Cube 的 startup_stm32h723xx.s。

## 板级前提

MR_CUBE_ROOT 指向完整 Cube 工程，当前验证 D:/STM32Projects_HAL/board-H7/zzrobot。只读 Core、Drivers、Middlewares 和启动汇编，不编译外部旧 main/freertos/User，不修改外部工程。

不是任意 Cube 工程都能直接使用：当前要求 H723XG、24 MHz HSE、相同 GPIO/SPI2/TIM/UART/FDCAN 句柄与初始化函数、TIM2 的 1 MHz/999 HAL 时基、1000 Hz FreeRTOS/CMSIS-RTOS2 和对应引脚。换板应调整 board/Cube 及仍受板型限制的 BSP，不能只改路径。

## 构建

安装 CMake >= 3.22、GNU Arm Embedded 工具链及 Ninja/Make。无需 Python 或配置依赖包。建议从新的 build 目录开始，避免旧生成链路的缓存。

```powershell
cmake -S examples/h7_integration -B build/h7-manual -G Ninja `
  "-DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake" `
  "-DMR_CUBE_ROOT=D:/STM32Projects_HAL/board-H7/zzrobot" `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/h7-manual --parallel 4
arm-none-eabi-nm -u build/h7-manual/mr_h7_integration.elf
```

Windows 使用 MinGW Makefiles 时，将 -G Ninja 换为 -G "MinGW Makefiles"，并按实际位置提供 CMAKE_MAKE_PROGRAM。arm-none-eabi-gcc/g++/objcopy/size 应在 PATH。输出 ELF/BIN/map；nm -u 应无输出。命令不烧录。

默认还严格编译全部 35 个 User 生产实现，检查未链接可选驱动的语法/include；不表示所有组合均已链接或运行验证。公共头检查仍可用可选验证脚本：

```powershell
py -3 tools/check_user_headers_h7.py --board-root D:/STM32Projects_HAL/board-H7/zzrobot
```

## 如何修改与维护

- 改 PID/几何/零点/限位：编辑 robot_params.hpp，直接编译。
- 换遥测 UART：改 board/bindings.hpp，确保 Cube 已初始化、uart_endpoints.c 已登记；无需改驱动或平行能力清单。
- 换电机或设备连接：改对应 assembly/motor_config；模块保持统一接口。
- 改关节数：编译期修改类型和参数，并显式调整装配的电机、关节与输出指针，不使用动态关节列表。
- 改任务/业务：直接编辑 robot.cpp 或底盘任务示例，不需要模板语言。
- 新模块：先实现 User 接口，添加必要 CMake 直接依赖和最小示例，即可在 app 使用，不需要生成器登记。

## 内存与行为

Flash 从 0x08000000 起 1 MiB，数据/BSS/RTOS heap/应用静态栈/MSP 在 0x24000000 起 320 KiB AXI SRAM，DMA 不放 DTCM。启动汇编负责 data/BSS 与 C++ 构造。Cache 显式关闭，DMA hook 只接受实际 AXI 区间；开启 Cache 要另行设计与验证。

应用线程栈/控制块静态提供：robot 32 KiB；独立底盘任务 4 KiB。外部 FreeRTOS heap 仍为 72 KiB，供 BSP 队列等现有分配使用，链接另预留 newlib heap 8 KiB 与 MSP 16 KiB。静态栈不会减少现有 heap 预留，因此本轮 RAM 总量比原动态任务版本增加；项目可在实测后调整 heap/栈预算，不能据此宣称全工程无堆。

UART/PWM/GPIO/SPI 在 main 的 Cube 初始化后登记；其他 BSP/设备注册在任务内。控制对象静态持有，注册不使能。默认 app::params::kAllowMotorTx=false，端口拒绝电机 TX，遥测仍工作。若显式修改发送许可，仍须新的 request_run 请求及真实有效反馈才能使能；没有串口使能命令。故障停止并锁住，需要撤销请求后重新请求。示例仅零底盘速度与机械臂保持。

参数均为演示值；DM MIT 量程必须匹配固件，external_ratio 仅表示额外机构减速；厂家内部减速已体现在 DM/LZ 协议物理量，不重复相乘。机械臂无重力补偿。

## 验证边界

手写装配使用同一代码参与主机真实电机协议集成回归；HAL/RTOS 通过目标固件编译与链接验证。未烧录、未测量栈高水位、任务时序、DMA/IRQ 或真实故障停机；仍需对应板级验收。其他 BSP 固定映射见 [核查表](../../docs/BSP_PORTABILITY_AUDIT.md)。

PWM 清单与用途分开：pwm_endpoints.c 只登记 tim12_ch2/tim3_ch4/tim1_ch1/tim1_ch3，bindings.hpp 选择蜂鸣器/加热/舵机用途。共享 TIM1 不允许单路改频。登记不启动输出；详见 [PWM 接入](../../docs/PWM_BINDINGS.md)。

GPIO/SPI 清单同样由 io_endpoints.c 提供；PC10/PC11 电磁阀与 PA2/PA0 继电器分别登记。已登记 EXTI 线先屏蔽，安装回调后逐线使能；board 为低电平有效的 BMI088 选择下降沿。当前 robot.cpp 未启动 IMU 采集服务，设备的手写接法见 [GPIO/SPI 接入](../../docs/GPIO_SPI_BINDINGS.md)。

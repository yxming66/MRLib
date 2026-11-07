# MR_TestLib

面向 STM32 机器人项目的 C/C++ 用户代码测试库，包含底层接口、设备驱动、控制组件、机械臂与底盘算法，以及应用集成示例。

当前阶段为**测试与协作开发版本**。本仓库提供 `User` 代码集合；接入具体硬件时，需要配套 HAL、RTOS、芯片启动文件、链接脚本及板级构建配置。[H723 集成示例](examples/h7_integration/README.md)已提供可完整链接的板级基线，默认禁止电机发送，尚未完成实机验收。

## 代码结构

| 目录 | 内容 |
| --- | --- |
| `User/bsp` | CAN/FDCAN、UART、GPIO、PWM、SPI、时间与内存等底层接口 |
| `User/component` | 控制器、滤波器、矩阵、轨迹、容器和命令中心等组件 |
| `User/device` | 电机、关节、轮组、传感器与通信设备 |
| `User/robotics` | 机械臂、底盘与旋转/变换数学算法 |
| `User/module` | 面向机器人功能的组合模块与适配层 |
| `User/task` | 项目任务边界说明；旧整机任务已移除 |
| `docs` | 来源记录、开发与验证说明 |

依赖方向应尽量保持：任务 → 模块 → 设备/算法 → 通用组件；底层 BSP 提供硬件接口。通用算法不要反向依赖任务、HAL 或具体电机。

## 设计目标

优先保证库易读、易组合、易维护。User 提供完整的普通 C/C++ 接口，项目通过 board、app、参数文件和任务代码完成装配；CMake 自动传递所选能力的编译依赖。不需要勾选复制源文件，也不要求 Python、YAML 或生成器。

module 保留统一接口：底盘注册时选择 robotics 模型、绑定 WheelDevice；机械臂使用编译期固定的 Arm<N>，项目静态持有电机/关节，模型与控制存储不使用堆分配。任务、遥控和整机业务归项目所有。

新增模块只维护功能、配置类型、CMake 直接依赖、最小手写示例与适用验证，不需要同步维护生成模板或 Schema。旧 YAML 配置/装配/任务生成与 CLI 已删除。未来自动配置可以在稳定接口上单独实现，不作为库使用前提。详见 [架构与维护工作](docs/ARCHITECTURE.md)。

## 开始使用

UART 已支持 board 硬件登记与项目代码用途分配，H7 MRLink 和 DR16 不再固定 RC/PC 串口。已有项目升级需要迁移 C API，见 [UART 接入](docs/UART_BINDINGS.md)。PWM 也已改为 board 登记，详见 [PWM 接入](docs/PWM_BINDINGS.md)。SPI/GPIO/FDCAN 等仍有旧板型映射，当前边界见 [BSP 核查](docs/BSP_PORTABILITY_AUDIT.md)。

```sh
git clone https://github.com/yxming66/MR_TestLib.git
cd MR_TestLib
```

阅读 [协作规范](CONTRIBUTING.md)，按需使用对应组件。机械臂算法说明见 [robotics/arm](User/robotics/arm/README.md)。

### 运行机械臂主机测试

Windows 安装支持 C++17 的 `g++`，确保在 PATH 中可用，在仓库根目录运行 PowerShell：

```powershell
.\User\robotics\arm\tests\run_host_tests.ps1
```

Linux 或其他具备 g++ 的环境：

```sh
mkdir -p build/host_tests
g++ -std=c++17 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -IUser \
  User/robotics/arm/tests/arm_host_tests.cpp \
  User/robotics/core/rotation.cpp User/robotics/core/transform.cpp \
  -o build/host_tests/arm_host_tests
./build/host_tests/arm_host_tests
```

### 接入 STM32 工程

1. 在独立板级工程中准备对应的 HAL、RTOS 和外设初始化。
2. 按实际依赖添加所需 `.c/.cpp` 与头文件路径，C++ 组件使用 C++17。
3. 显式列出源文件，排除测试和尚未迁移的旧专用控制器，不要递归编译整个仓库。
4. 核对任务入口、引脚、总线、设备 ID、单位、方向、零点、限位及线程/中断使用约束。
5. 完成编译和对应实机验证后，记录板型、工具链、提交号及结果。

旧整机任务已移除；当前 H723 集成示例提供独立构建入口，已完成选定装配的完整固件链接及全部 User 生产实现严格编译。其他功能组合和板上运行仍需分别验证。审查方案与已完成批次见 [User 重构方案](docs/USER_REFACTOR_PLAN.md) 和 [重构进度](docs/USER_REFACTOR_PROGRESS.md)。

### 主机构建与 CMake target

仓库提供一个不依赖 STM32 HAL 的主机构建入口，用于验证纯算法和后续可复用组件：

```powershell
cmake -S . -B build/cmake-host -G Ninja -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

当前已有 32 项主机回归，覆盖 C11/C++17 基础组件、共享 PID 控制律、滤波频率响应、计时、姿态/校准/告警服务、RM/DM/LZ 协议 fake-CAN 路径、机械臂、统一底盘运动学和统一底盘模块（含反馈超时及计时回绕契约）。STM32 工程可以用 `add_subdirectory()` 引入本库，再通过 `target_link_libraries()` 选择 `mr_robotics_arm`、`mr_robotics_chassis`、`mr_module_chassis` 等 target；应用工程仍保留自己的 HAL、RTOS、启动文件、链接脚本和板级适配。详细说明见 [构建入口](docs/BUILD.md)，新增服务的接入契约见 [基础服务接口](docs/USER_SERVICES.md)。

库的分层、统一底盘接口和迁移边界见 [架构说明](docs/ARCHITECTURE.md) 与 [统一底盘模块](User/module/chassis/README.md)。

### 新建项目

先准备可构建的 Cube 工程，引入完整库，在 CMake 链接所用 target；board 连接硬件，app 参数文件填写标定值，普通 C++ 创建并连接对象，任务显式调度。新增功能可以立即由 app 调用，不受自动配置工具支持范围限制。

可阅读或复制以下示例：

- [普通 CMake 消费项目](examples/host_project/README.md)：最小 target 引入与链接，无 Python。
- [H723 完整工程](examples/h7_integration/README.md)：手写底盘/Arm<3>/遥测装配、参数和板级启动。
- [底盘周期任务](examples/chassis_task/README.md)：直接修改周期、栈和业务的小型 FreeRTOS 示例。

示例复制后归项目维护，不存在再次生成覆盖。板级前提和接入步骤见 [板级集成](docs/BOARD_INTEGRATION.md)。

## 多人开发

- 先开 Issue，确认负责人、改动范围和验收方式。
- 从 `main` 创建 `feat/...`、`fix/...` 或 `docs/...` 短期分支。
- 提交 PR，至少一名其他成员评审，通过适用测试后由维护者合入。
- 影响硬件行为的改动补充实机证据；尚未验证的内容明确标注，不能作为稳定发布依据。
- 不把各人的整套机器人项目长期并入库，通过适配层接入公共接口。

具体流程、角色、接口规则和 GitHub 设置见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 当前验证与来源

2026-10-26 初始整理时，RC_02/User 为主体，合入 zzrobot/User 的 10 个机械臂及共享旋转数学差异文件，共 283 个文件。当时保存的旧版 User_code 参考目录已在后续清理中移除，历史来源可从 Git 记录追溯。

初始导入完成了文件哈希核对和机械臂主机回归测试；不代表整个库已通过固件编译或实机测试。CI 配置运行 CMake 主机回归和普通消费项目构建，不覆盖 STM32 固件及实机验证。

[SOURCE_MANIFEST.csv](SOURCE_MANIFEST.csv) 和 [BASELINE_MANIFEST.csv](BASELINE_MANIFEST.csv) 是**初始导入快照**，后续开发以 Git 提交记录为准，不要求每次改动重算历史清单。详细记录见 [导入说明](docs/IMPORT_NOTES.md)。

## 许可

当前未指定开源许可证。上传 GitHub 不自动授予第三方开源使用权；引入外部代码时需保留其版权与许可证。正式对外开放前，由仓库维护者确认项目许可及第三方依赖许可。

# MR_TestLib

面向 STM32 机器人项目的 C/C++ 用户代码测试库，包含底层接口、设备驱动、控制组件、机械臂与底盘算法，以及应用集成示例。

当前阶段为**测试与协作开发版本**。本仓库提供 `User` 代码集合，不是可以直接烧录的完整 STM32 工程；接入具体硬件时，需要配套 HAL、RTOS、芯片启动文件、链接脚本及板级构建配置。

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

## 长期设计目标

这个库的目标是让新机器人项目能从板级工程模板快速组装，而不是让开发者手动挑选大量源文件。项目用 YAML 声明板级适配、外设、设备型号、机器人模型、模块参数和任务配置；命令行工具解析依赖并检查配置，逐步生成项目装配代码和构建入口。生成物应与手写业务代码分开，重复生成不能覆盖用户代码；自定义外设通过清晰的适配接口接入。

`module` 面向机器人能力提供统一接口。例如底盘或云台模块不为每个型号复制一套控制入口；注册时选择具体类型，再连接 `robotics` 层相应的模型和算法。项目任务负责调度和业务逻辑，不承担手工拼装所有库文件的工作。

期望的新建项目流程是：准备 STM32 板级工程，在项目配置中选择所需能力，运行工具检查依赖并生成装配，再编译、烧录和标定。工具应指出缺少或冲突的依赖；开发者不应靠反复勾选文件来修复 include 缺失，也不应担心重新生成覆盖手写代码。VS Code 插件可以在命令行和配置格式稳定后提供图形入口，核心构建流程不依赖插件。

当前 CLI 仍处于基础阶段：可以选择已登记的 CMake target、生成主机项目脚手架并检查公开头文件依赖；它还不能完整生成 STM32 外设注册、设备装配、模块配置或任务入口。上述目标是后续架构与工具优化的验收方向，不代表当前功能已经具备。详见[架构说明](docs/ARCHITECTURE.md)和[项目配置说明](docs/PROJECT_CONFIG.md)。

## 开始使用

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

原工程构建配置并未包含在本库中。当前处于接口重构阶段，旧整机任务已移除，旧专用底盘控制器和机械臂模块仍有失效依赖；不能用主机测试通过推断整个 `User` 已能生成固件。审查方案与已完成批次见 [User 重构方案](docs/USER_REFACTOR_PLAN.md) 和 [重构进度](docs/USER_REFACTOR_PROGRESS.md)。

### 主机构建与 CMake target

仓库提供一个不依赖 STM32 HAL 的主机构建入口，用于验证纯算法和后续可复用组件：

```powershell
cmake -S . -B build/cmake-host -G Ninja -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

当前已有 13 项主机回归，覆盖 C11/C++17 基础组件、共享 PID 控制律、滤波频率响应、计时、姿态/校准/告警服务、RM/DM/LZ 协议 fake-CAN 路径、机械臂、统一底盘运动学和统一底盘模块（含反馈超时及计时回绕契约）。STM32 工程可以用 `add_subdirectory()` 引入本库，再通过 `target_link_libraries()` 选择 `mr_robotics_arm`、`mr_robotics_chassis`、`mr_module_chassis` 等 target；应用工程仍保留自己的 HAL、RTOS、启动文件、链接脚本和板级适配。详细说明见 [构建入口](docs/BUILD.md)，新增服务的接入契约见 [基础服务接口](docs/USER_SERVICES.md)。

库的分层、统一底盘接口和迁移边界见 [架构说明](docs/ARCHITECTURE.md) 与 [统一底盘模块](User/module/chassis/README.md)。

### 新建项目

Python 3.10+、CMake 3.22+ 和 C++17 编译器就绪后，可以生成一个只选择所需库 target 的普通 CMake 工程：

```powershell
py -3 tools/mrctl.py init build/demo --name robot_demo `
  --library ../.. `
  --targets mr_robotics_arm,mr_module_chassis
py -3 tools/mrctl.py configure build/demo
py -3 tools/mrctl.py build build/demo
```

工程配置保存在 `project.yaml`，生成器只写入不存在的 starter 文件；添加或删去库 target 后再次 configure 即可。完整配置格式、现有限制与 STM32 工程的接入边界见 [项目快速构建](docs/PROJECT_CONFIG.md)。这是 CMake 工程脚手架与主机接口检查，不代替板级 HAL、RTOS、启动文件和链接脚本。

## 多人开发

- 先开 Issue，确认负责人、改动范围和验收方式。
- 从 `main` 创建 `feat/...`、`fix/...` 或 `docs/...` 短期分支。
- 提交 PR，至少一名其他成员评审，通过适用测试后由维护者合入。
- 影响硬件行为的改动补充实机证据；尚未验证的内容明确标注，不能作为稳定发布依据。
- 不把各人的整套机器人项目长期并入库，通过适配层接入公共接口。

具体流程、角色、接口规则和 GitHub 设置见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 当前验证与来源

2026-10-26 初始整理时，RC_02/User 为主体，合入 zzrobot/User 的 10 个机械臂及共享旋转数学差异文件，共 283 个文件。当时保存的旧版 User_code 参考目录已在后续清理中移除，历史来源可从 Git 记录追溯。

初始导入完成了文件哈希核对和机械臂主机回归测试；不代表整个库已通过固件编译或实机测试。CI 配置运行全部 CMake 主机回归和项目 CLI 检查，不覆盖 STM32 固件及实机验证。

[SOURCE_MANIFEST.csv](SOURCE_MANIFEST.csv) 和 [BASELINE_MANIFEST.csv](BASELINE_MANIFEST.csv) 是**初始导入快照**，后续开发以 Git 提交记录为准，不要求每次改动重算历史清单。详细记录见 [导入说明](docs/IMPORT_NOTES.md)。

## 许可

当前未指定开源许可证。上传 GitHub 不自动授予第三方开源使用权；引入外部代码时需保留其版权与许可证。正式对外开放前，由仓库维护者确认项目许可及第三方依赖许可。

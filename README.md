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
| `User/task` | 任务入口和项目集成示例 |
| `_reference/User_code` | 旧版可信库快照，仅用于参考与按需迁移 |
| `docs` | 来源记录、开发与验证说明 |

依赖方向应尽量保持：任务 → 模块 → 设备/算法 → 通用组件；底层 BSP 提供硬件接口。通用算法不要反向依赖任务、HAL 或具体电机。

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
3. 显式列出源文件，排除 `tests`、`_reference` 和未使用的任务示例，不要递归编译整个仓库。
4. 核对任务入口、引脚、总线、设备 ID、单位、方向、零点、限位及线程/中断使用约束。
5. 完成编译和对应实机验证后，记录板型、工具链、提交号及结果。

部分 `.c` 文件在原工程中按 C++ 编译，例如 `module/arm_simple.c` 和 `module/camera_yaw.c`，集成时需要显式设置源文件语言。原工程构建配置并未包含在本库中。

### 主机构建与 CMake target

仓库提供一个不依赖 STM32 HAL 的主机构建入口，用于验证纯算法和后续可复用组件：

```powershell
cmake -S . -B build/cmake-host -G Ninja -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

当前已有机械臂、统一底盘运动学和统一底盘模块主机测试。STM32 工程可以用 `add_subdirectory()` 引入本库，再通过 `target_link_libraries()` 选择 `mr_robotics_arm`、`mr_robotics_chassis`、`mr_module_chassis` 等 target；应用工程仍保留自己的 HAL、RTOS、启动文件、链接脚本和板级适配。详细说明见 [构建入口](docs/BUILD.md)。

库的分层、统一底盘接口和迁移边界见 [架构说明](docs/ARCHITECTURE.md) 与 [统一底盘模块](User/module/chassis/README.md)。

## 多人开发

- 先开 Issue，确认负责人、改动范围和验收方式。
- 从 `main` 创建 `feat/...`、`fix/...` 或 `docs/...` 短期分支。
- 提交 PR，至少一名其他成员评审，通过适用测试后由维护者合入。
- 影响硬件行为的改动补充实机证据；尚未验证的内容明确标注，不能作为稳定发布依据。
- 不把各人的整套机器人项目长期并入库，通过适配层接入公共接口。

具体流程、角色、接口规则和 GitHub 设置见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 当前验证与来源

2026-10-26 初始整理时，RC_02/User 为主体，合入 zzrobot/User 的 10 个机械臂及共享旋转数学差异文件，共 283 个文件。旧版 User_code 保留为参考快照。

初始导入完成了文件哈希核对和机械臂主机回归测试；不代表整个库已通过固件编译或实机测试。CI 当前只覆盖机械臂主机测试。

[SOURCE_MANIFEST.csv](SOURCE_MANIFEST.csv) 和 [BASELINE_MANIFEST.csv](BASELINE_MANIFEST.csv) 是**初始导入快照**，后续开发以 Git 提交记录为准，不要求每次改动重算历史清单。详细记录见 [导入说明](docs/IMPORT_NOTES.md)。

## 许可

当前未指定开源许可证。上传 GitHub 不自动授予第三方开源使用权；引入外部代码时需保留其版权与许可证。正式对外开放前，由仓库维护者确认项目许可及第三方依赖许可。

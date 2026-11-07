# 构建入口

仓库提供两种 CMake 使用方式：仓库自身的主机构建用于回归测试；机器人工程使用已登记的 `mr::...` target 获得组件和传递依赖。主机构建不依赖 STM32 HAL，也不生成可烧录固件。

## 构建和回归测试库

```powershell
cmake -S . -B build/cmake-host -G Ninja `
  -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

当前包括 C11 基础组件、C++17 基础组件、滤波、计时、姿态/校准/告警服务、RM/DM/LZ 电机协议 fake-CAN 路径、C PID 核心、C++ PID/旧参数适配、机械臂、底盘运动学和统一底盘模块以及矩阵/轨迹、固定存储和几何有效性共 25 项主机回归测试。测试直接编译实际 C AHRS、滤波、PID 适配、CRC16、C++ timer 和 RM/DM/LZ protocol；电机总线使用注入的 CanPort 测试替身，不需要 STM32 HAL/RTOS。

滤波测试检查截止频率处的 -3 dB 响应、陷波中心衰减、C/C++ 一致性与变周期切换；服务测试检查重复/过期反馈、实际采样间隔、重新校准和告警保持。测试未验证真实 CAN 外设、STM32 固件链接与板级运行。

PID 测试使用独立 PI 预期值验证积分输出限幅、饱和恢复和斜率限制；验证微分首帧、显式 dt、算术/滤波错误时保持控制历史，以及三种微分模式下旧 C 入口与 C++ 接口的一致性。底盘模块测试确认非法 PID/反馈超时配置在设备注册前被拒绝，反馈超时触发安全放松，uint32 毫秒时钟回绕可正确处理。

RM 命令组测试检查 M2006/M3508/GM6020 的地址映射、四槽大端帧编码、不同帧组缓存隔离和非法节点/槽位拒绝。fake-CAN 协议测试编译真实 RM/DM/LZ 协议源，通过 CanPort 注入发送/接收，检查零电流 Relax、发送失败重试、失败命令缓存隔离、地址边界、非有限输入拒绝及 enable/disable 状态更新。测试替身不代表真实 BSP、固件或硬件行为已经验证。

## 新建主机项目

```powershell
py -3 tools/mrctl.py init build/demo --name robot_demo `
  --library ../.. `
  --targets mr_robotics_arm,mr_module_chassis
py -3 tools/mrctl.py configure build/demo
py -3 tools/mrctl.py build build/demo
```

`project.yaml` 保存选定的 target；`mrctl configure` 将它们对应的 public header 检查生成到 build 目录，再运行普通的 CMake 配置。生成文件只放在构建目录，`init` 不覆盖已有源文件。更多 target、配置校验和限制见 [项目快速构建](PROJECT_CONFIG.md)。

## 作为子目录使用

STM32 应用的既有 CMake 工程可把本库作为 Git submodule 引入，再链接所需功能：

```cmake
add_subdirectory(external/MR_TestLib)
target_link_libraries(robot_firmware PRIVATE
    mr::robotics_arm
    mr::module_chassis
)
```

当前公共 target：

| Target | 用途 |
| --- | --- |
| `mr::component_math` | 共用 C11 标量核心、薄 C++17 入口和其他 C++ 数学工具 |
| `mr::component_filter` | 通用滤波器头文件 |
| `mr::component_controller` | 通用控制器头文件 |
| `mr::component_trajectory` | 轨迹规划组件 |
| `mr::robotics_core` | 旋转与变换类型和实现 |
| `mr::robotics_chassis` | 底盘运动学 |
| `mr::device_wheel_interface` | 电机型号无关的轮子设备接口 |
| `mr::module_chassis` | 统一底盘控制接口 |
| `mr::robotics_arm` | 机械臂算法 |

库 target 会传递 `User` 头文件路径和已声明的依赖。目标工程仍负责 HAL、RTOS、启动文件、链接脚本、BSP 与具体设备适配。

timer 已登记为 mr::component_timer；姿态估计与告警服务尚未登记为 CLI 可选 target；手工接入时显式添加所需实现。C++ 低通/陷波和告警为头文件实现；C 滤波 API 需编译 `User/component/filter/filter.c`，timer 需 `User/component/timer.cpp`，姿态估计需 `User/component/ahrs.c`，并设置 `User` 为 include 路径。详细接口、单位和迁移变化见 [基础服务接口](USER_SERVICES.md)。

共享 PID 核心为 `component/controller/pid_core.h`，C11 和 C++17 均可使用，无需额外源文件；C++ `pid.hpp` 通过既有 `mr::component_controller` 使用。旧 `PID_Init/Calc/Reset` 入口需显式编译 `User/component/pid.c`；PID 自身不再需要链接 `filter.c`，滤波计算直接复用头文件核心。

本次模块边界调整新增公共目标：

| Target | 用途 |
| --- | --- |
| `mr::device_joint_interface` | 通用执行器与关节接口 |
| `mr::module_arm` | 外部装配的固定关节数 `Arm<N>` |
| `mr::module_audio` | C11 音符播放状态机，无设备依赖 |
| `mr::module_cloudmusic` | 外部播放列表与曲目控制，依赖 module_audio |

新增机械臂模块和音乐主机回归；三轴装配示例也参与测试编译。曲谱无需加入公共依赖，应用显式添加 `resources/music` 中的单曲源文件，见 `examples/music/README.md`。


MRLink 可移植协议使用导出目标 `mr_component_mrlink`，主机测试不使用 HAL/CMSIS stub。
通道生命周期回归额外编译实际 UART/SPI/BMI088/DR16 源，用模拟 HAL 检查调用次序和所有权。
可选真实 H723/Cortex-M7 编译检查：

```powershell
py -3 tools/check_user_io_h7.py --board-root D:/STM32Projects_HAL/board-H7/zzrobot
```

该检查只读参考工程，以 `-Werror`、严格 C11/C++17、无异常/RTTI 编译 13 份实际源及装配例子。
不执行完整固件链接、烧录、Cache 实测或实时性验收。

作为子目录引入库时，父工程应先声明 `project(... LANGUAGES C CXX)`，因为库含有 C 与 C++ 导出目标。

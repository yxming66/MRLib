# 构建入口

当前仓库已经提供一个不依赖 STM32 HAL 的主机构建入口。它用于验证库内的纯算法和组件，不会生成可以直接烧录的固件。

## 快速构建

在仓库根目录执行：

```powershell
cmake -S . -B build/cmake-host -G Ninja `
  -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

当前会构建并运行：

- `mr_arm_host_tests`：机械臂主机回归测试。
- `mr_chassis_host_tests`：统一底盘运动学选择器测试。

可以关闭某类测试，验证库 target 自身是否能被另一个工程引用：

```powershell
cmake -S . -B build/cmake-host -G Ninja `
  -DMR_TESTLIB_BUILD_HOST_TESTS=OFF
```

## 作为子目录使用

STM32 工程可以把本库作为 Git submodule 或固定版本的源码依赖：

```cmake
add_subdirectory(external/MR_TestLib)

target_link_libraries(robot_firmware PRIVATE
    mr_robotics_arm
    mr_robotics_chassis
)
```

库 target 会传递 `User` 头文件路径和已声明的依赖。应用工程仍然负责 HAL、RTOS、启动文件、链接脚本和板级适配。

当前提供的公共 target：

| Target | 用途 |
| --- | --- |
| `mr_component_math` | 纯 C++ 数学工具头文件 |
| `mr_component_filter` | 通用滤波器头文件 |
| `mr_component_controller` | PID 等通用控制器头文件 |
| `mr_robotics_chassis` | 底盘运动学头文件和统一模型选择器 |
| `mr_module_chassis` | 统一底盘控制接口和轮子设备绑定 |
| `mr_device_wheel_interface` | 不依赖具体电机型号的轮子设备接口 |
| `mr_robotics_arm` | 机械臂算法头文件及旋转/变换实现 |

底盘模型注册示例：

```cpp
#include "robotics/chassis/chassis_kinematics.hpp"

mr::robotics::chassis::ChassisKinematics kinematics;
mr::robotics::chassis::ChassisKinematicsConfig config{};
config.model = mr::robotics::chassis::ChassisModel::kMecanum;
config.geometry.wheelbase_m = 0.25f;
config.geometry.trackwidth_m = 0.20f;

if (!kinematics.Configure(config)) {
  // 参数非法，保持安全状态
}
```

选择 `kFrontOmniRearMecanum` 时，调用代码保持不变，只有运动学模型发生变化。该选择器使用固定存储，不使用动态内存，也不依赖具体电机型号。

## 下一步

后续会按这个构建边界继续增加：

1. `module/chassis` 的统一控制接口 target。
2. device binding 和板级适配 target。
3. 参考项目模板及项目级配置校验。
4. `mrctl configure/build` 命令行入口。

在这些入口稳定前，不建议从整个 `User` 目录递归收集源文件。项目应显式链接模块 target，由 target 依赖图决定参与编译的文件。

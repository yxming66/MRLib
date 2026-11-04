# 构建入口

仓库提供两种 CMake 使用方式：仓库自身的主机构建用于回归测试；机器人工程使用已登记的 `mr::...` target 获得组件和传递依赖。主机构建不依赖 STM32 HAL，也不生成可烧录固件。

## 构建和回归测试库

```powershell
cmake -S . -B build/cmake-host -G Ninja `
  -DMR_TESTLIB_BUILD_HOST_TESTS=ON
cmake --build build/cmake-host
ctest --test-dir build/cmake-host --output-on-failure
```

机械臂、底盘运动学和统一底盘模块均有主机回归测试。

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
| `mr::component_math` | 纯 C++ 数学工具头文件 |
| `mr::component_filter` | 通用滤波器头文件 |
| `mr::component_controller` | 通用控制器头文件 |
| `mr::component_trajectory` | 轨迹规划组件 |
| `mr::robotics_core` | 旋转与变换类型和实现 |
| `mr::robotics_chassis` | 底盘运动学 |
| `mr::device_wheel_interface` | 电机型号无关的轮子设备接口 |
| `mr::module_chassis` | 统一底盘控制接口 |
| `mr::robotics_arm` | 机械臂算法 |

库 target 会传递 `User` 头文件路径和已声明的依赖。目标工程仍负责 HAL、RTOS、启动文件、链接脚本、BSP 与具体设备适配。

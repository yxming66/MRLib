# 项目配置与快速构建

本文描述现有 **v1 主机 target 配置**。完整机器人参数统一采用新的 [`config.yaml` v2 范本](../examples/h7_integration/config.yaml)，字段规范、校验命令和生成边界见 [机器人参数配置](ROBOT_CONFIG.md)。v2 已支持独立校验，尚未接入 `mrctl configure/build` 的固件生成；两种格式不可混用。

`project.yaml` 是桌面构建工具读取的简明配置：项目名、库的位置和所选库 target。应用只列出直接需要的模块；公共 target、入口头文件和直接依赖登记在 [`cmake/targets.json`](../cmake/targets.json)，CMake 和 `mrctl.py` 共用这份清单，构建图负责传递头文件目录和模块依赖。JSON Schema 只描述配置格式，并由测试检查 target 枚举与清单同步。整个 CLI 只用 Python 标准库。

需要 Python 3.10+、CMake 3.22+ 和支持 C++17 的编译器。系统安装 Ninja 后，工具自动选择它；否则由 CMake 选择默认生成器。工具不下载编译器或其他依赖。

## 创建并验证一个主机项目

在仓库根目录执行；本例将示例生成在 `build/demo`，库路径 `../..` 相对于新项目目录：

```powershell
py -3 tools/mrctl.py init build/demo --name robot_demo `
  --library ../.. `
  --targets mr_robotics_arm,mr_module_chassis
py -3 tools/mrctl.py configure build/demo
py -3 tools/mrctl.py build build/demo
```

`init` 创建项目配置、CMake 入口、`src/main.cpp` 应用起点及忽略 build 目录的 `.gitignore`。如果起点文件已经存在，工具会保留其内容。`configure` 根据配置展开传递依赖，生成 build 目录中的公共头文件编译探针，并把已选库 target 传给 CMake。命令会打印自动补入的依赖。改选 target 后再次 configure 和 build，即可更新检查和目标依赖；未选中的库 target 不会被项目构建。

项目也可以在 VS Code 中作为普通 CMake 项目使用。首次打开前应先运行 `mrctl.py configure`，使配置对应的 compile check 已生成。手动修改选用的 target 后，再通过工具重新 configure 和 build。

## 项目配置

```yaml
version: 1
project:
  name: robot_demo
  library: ../../external/MR_TestLib
  targets:
    - mr_robotics_arm
    - mr_module_chassis
```

当前 CLI 接受该格式的缩进映射及 block 列表；不要写 YAML anchor、嵌套自定义对象或多行 scalar。`name` 是简单的 CMake target 名。`library` 必须是相对于项目目录的 MR_TestLib 路径。由 Git 管理的独立机器人项目可以用固定提交的 submodule 引用库；提交配置、submodule 指针及业务代码，不提交 `build/`。

已登记的接口 target：

| 配置 target | 范围 | 传递依赖 |
| --- | --- | --- |
| `mr_component_math` | 数学与矩阵 | — |
| `mr_component_filter` | 滤波算法 | math |
| `mr_component_controller` | 控制器 | filter、math |
| `mr_component_trajectory` | 轨迹 profile | math |
| `mr_robotics_core` | 旋转、变换与对应实现 | math |
| `mr_robotics_chassis` | 底盘运动学 | math |
| `mr_device_wheel_interface` | 电机型号无关的轮子接口 | — |
| `mr_module_chassis` | 统一底盘控制模块 | robotics chassis、controller、wheel interface |
| `mr_robotics_arm` | 机械臂算法 | math、trajectory、robotics core |
| `mr_device_joint_interface` | 通用关节/执行器接口 | — |
| `mr_module_arm` | 外部装配的 `Arm<N>` | robotics arm、joint interface |
| `mr_module_audio` | 非阻塞音符播放 | — |
| `mr_module_cloudmusic` | 外部播放列表和控制事件 | module audio |

配置使用普通 target 名称（如下划线格式）；CMake 公开稳定别名 `mr::component_math`、`mr::robotics_arm` 等。直接依赖表达模块需要的能力，其他依赖从 target 图传递。

## STM32 固件工程

`mrctl configure/build` 是可验证选择接口的**主机**项目起步流程，不会生成或烧写固件。已有板级 CMake 工程可以选择同样的库 target：

```cmake
add_subdirectory(external/MR_TestLib)
target_link_libraries(robot_firmware PRIVATE
    mr::robotics_arm
    mr::module_chassis
)
```

固件工程继续由板级 toolchain、HAL、RTOS、启动文件、链接脚本、BSP 和设备适配来提供余下配置与实现。库不会猜测、复制或递归扫描这些专属文件。

# 先看一个底盘任务

本示例使用 FreeRTOS 内核和 CMSIS-RTOS2 接口，复用 H7 示例的 Cube/board 接入。只运行一个底盘任务，不创建机械臂或通信任务。默认关闭电机发送和运行请求；示例参数不代表真实机器人标定。

建议按下面的顺序打开文件：

1. **自己改参数：** [config.yaml](config.yaml) 最后的 `tasks.chassis`。周期 2000 μs、栈 4096 字节、普通优先级、入口 `app.chassis_cycle`。
2. **自己写业务：** [app/chassis_task.cpp](app/chassis_task.cpp)。`Init()` 注册一次；`Step(dt_s)` 每周期调用一次，读取命令并执行底盘控制；`Fault(reason)` 处理初始化/控制失败或周期超时。底部 `BindPeriodicTask()` 将 YAML 入口名连接到这些函数。这里都是手写代码，生成器不会修改。
3. **自动生成配置：** 执行下述命令后打开 `build/chassis-task-preview/generated/robot_tasks.hpp` 和 `robot_tasks.cpp`。前者只有名称、周期、栈、优先级和入口标识，后者把这些配置及 app 提供的回调交给固定执行器。
4. **普通 C++ 实现调度：** [User/task/periodic_task.hpp](../../User/task/periodic_task.hpp)。创建线程、调用 Init、循环调用 Step、`osDelayUntil()` 等都写在这里；改调度机制就改这个文件，不需要改 Python 模板。

```powershell
build/config-venv/Scripts/python.exe tools/mrctl.py generate examples/chassis_task/config.yaml `
  --module chassis --task chassis --output-dir build/chassis-task-preview/generated
```

生成结果中最主要的配置就是：

```cpp
inline constexpr mr::task::PeriodicTaskConfig kTaskConfig{
    "chassis", 2000u, 4096u, osPriorityNormal};
inline constexpr const char* kTaskEntry = "app.chassis_cycle";
```

流程：`YAML → Python 校验/生成参数 → PeriodicTask.Create → Init → Step → 周期等待`。生成器在 [tools/task_generator.py](../../tools/task_generator.py)；它只生成上述连接代码，控制逻辑和调度机制不藏在 Python 中。当前不提供自动创建 app 文件的命令，app 示例是已经写好的参考文件。

## 编译这个示例

从库根目录运行，需已有 GNU Arm 工具链、CMake、Python 配置依赖及完整 H723 Cube 工程。下面与本仓库已有构建环境一致；其他电脑调整路径和构建器。

```powershell
cmake -S examples/h7_integration -B build/h7-chassis-task -G "MinGW Makefiles" `
  "-DCMAKE_MAKE_PROGRAM=D:/Drivers/MSYS2/ucrt64/bin/mingw32-make.exe" `
  "-DCMAKE_TOOLCHAIN_FILE=D:/STM32Projects_HAL/Mrobot_Test_Version/MR_TestLib/examples/h7_integration/arm-none-eabi.cmake" `
  "-DMR_CUBE_ROOT=D:/STM32Projects_HAL/board-H7/zzrobot" `
  "-DMR_CONFIG_PYTHON=D:/STM32Projects_HAL/Mrobot_Test_Version/MR_TestLib/build/config-venv/Scripts/python.exe" `
  -DMR_CHASSIS_TASK_EXAMPLE=ON
cmake --build build/h7-chassis-task --parallel 4
```

真正编译的生成物在 `build/h7-chassis-task/generated/`。修改 YAML 后只需再次 build；CMake 自动更新配置和任务代码。此构建复用 `h7_integration/board/main.c`，在 `osKernelInitialize()` 与 `osKernelStart()` 之间调用 `MR_CreateConfiguredTask()`，不创建原来的 RobotTask。不要同时在 Cube freertos.c 再创建相同任务。

## 自己主要改哪里

| 要修改的内容 | 文件 |
| --- | --- |
| 周期、栈、优先级及底盘参数 | `config.yaml` |
| 遥控命令来源、使能条件、业务控制和故障动作 | `app/chassis_task.cpp` |
| 通用线程创建、周期等待机制 | `User/task/periodic_task.hpp` |
| 新增一个可生成配置字段 | Schema、`tools/task_generator.py` 和对应 C++ 配置 |

`ReadCommand()` 当前仅读取调试器运行请求，速度给零；项目应替换为有同步和超时检查的命令快照。它不是跨线程 `volatile` 通信示例。使能前检查四轮新鲜反馈，注册不会自动使能。

首版只生成一个周期任务，要求其恰好拥有选中的模块且 services 为空；通信、事件等待任务、队列拓扑和多任务资源启动屏障尚未生成。任务通过 CMSIS 动态分配，需要 board 启用动态分配并提供足够 FreeRTOS heap。周期按真实 tick 频率检查，不能精确表示就拒绝创建，不静默取整。初始化失败、Step 失败、达到/超过下一周期或等待失败都会调用 Fault 后退出任务，不自动追赶或重启；示例 Fault 尝试发送零输出并记录失败，不能保证总线故障时实际制动。当前需要复位后重启该任务。

生成物受摘要清单保护，手改会拒绝覆盖。取消 `--task` 后旧任务文件保留但不再加入编译；重新启用时仍检查归属。创建入口只允许在内核启动前调用一次。回调在所属任务内执行，长期阻塞或任务完全卡死不在此周期检测覆盖范围内。

验证包括主机 CMSIS 调用契约（tick 回绕、精确周期、创建/初始化/控制/超时/等待失败）和真实 H723 固件编译链接；主机 stub 不模拟真实抢占。未进行烧录或板上验证，4 KiB 栈也尚未实机测量。

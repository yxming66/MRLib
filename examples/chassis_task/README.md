# 手写 FreeRTOS 底盘周期任务

这是可复制并直接修改的普通 C++ 示例，不读取 YAML、不生成文件。业务在 app/chassis_task.cpp，周期/栈/优先级在 app/task_params.hpp。为减少演示重复，直接复用相邻 H7 示例的 chassis_assembly.hpp、robot_params.hpp 和 board/bindings.hpp；迁移到独立项目时把它们放进本项目的 app/board。

文件职责：

- task_params.hpp：默认 2 ms、4096 字节、普通优先级。
- chassis_task.cpp：静态持有底盘，明确 Init/Step/Fault 和创建入口 App_CreateChassisTask。
- User/task/periodic_task.hpp：可选的小型 CMSIS-RTOS2 执行器，负责精确 tick 换算、delay-until 和故障退出，业务不藏在模板或 Python 中。

任务在 osKernelInitialize 后、osKernelStart 前创建。控制块和栈由项目静态提供，不回退到堆。初始化在任务内执行；模块注册不使能。Init/Step 失败、周期超时或等待失败时调用 Fault 并退出，无自动追赶周期或重启。

```powershell
cmake -S examples/h7_integration -B build/h7-manual-chassis -G Ninja `
  "-DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake" `
  "-DMR_CUBE_ROOT=D:/STM32Projects_HAL/board-H7/zzrobot" `
  -DMR_CHASSIS_TASK_EXAMPLE=ON
cmake --build build/h7-manual-chassis
```

先阅读 H7 README 的板级前提和工具链要求。应用发送许可在 H7 app/robot_params.hpp，默认 false；运行请求初始 false。ReadCommand 仅演示零速度和调试器请求，实际项目自行接入带超时的命令快照。volatile 不是跨任务同步。

复制到独立目录时，H7 CMake 配置加 `"-DMR_LIBRARY_ROOT=D:/path/to/MR_TestLib"`，并将本目录放在 H7 项目旁命名为 chassis_task。任务使用 H7 项目的 app/board include 路径，控制循环辅助头由 CMake 显式从库内 examples/device_execution 引入；完整复制与构建步骤见 [新项目接入](../../docs/NEW_PROJECT.md)。

修改周期后直接重新编译，PID 频率由同一 period_us 派生。增加队列、通知或多任务，直接改本项目代码；也可以不用执行器，手写完整任务循环。共享总线、启动顺序与故障恢复由项目明确设计。静态任务栈不代表底层 BSP 队列完全无堆。未测量板上栈高水位或时序，未烧录。

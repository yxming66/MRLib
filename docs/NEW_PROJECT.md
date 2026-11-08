# 从新目录接入项目

当前流程是：准备项目目录，引入完整库，链接所用 target，在 board/app 中手写配置和对象连接。无需运行 Python、填写 YAML 或生成代码。本文命令使用 PowerShell 和 Ninja；需要先将 CMake、Ninja、主机 GCC/G++ 或 GNU Arm 工具链加入 PATH。

## 1. 先验证独立主机项目

以下路径仅作示例，my_robot_host 应为新的目录。在任意工作目录执行：

```powershell
$libraryRoot = 'D:/STM32Projects_HAL/Mrobot_Test_Version/MR_TestLib'
$projectRoot = 'D:/STM32Projects_HAL/my_robot_host'
if (Test-Path -LiteralPath $projectRoot) { throw '请选择新的项目目录' }
New-Item -ItemType Directory -Path $projectRoot | Out-Null
Copy-Item -LiteralPath "$libraryRoot/examples/host_project/CMakeLists.txt" -Destination $projectRoot
Copy-Item -LiteralPath "$libraryRoot/examples/host_project/app" -Destination $projectRoot -Recurse
cmake -S $projectRoot -B "$projectRoot/build" -G Ninja "-DMR_LIBRARY_ROOT=$libraryRoot"
cmake --build "$projectRoot/build" --parallel 4
ctest --test-dir "$projectRoot/build" --output-on-failure
```

成功后 robot_app 应返回 0。该示例实际调用底盘运动学、robotics_core 的旋转实现及 C MRLink 初始化，检查 C/C++ 链接与传递依赖；不连接硬件，也不初始化机械臂设备。CMake 只需链接 module_arm、module_chassis、component_mrlink，无需手补其核心算法源和 include。

MR_LIBRARY_ROOT 指向库仓库，既可用现有 checkout，也可用项目 external/MR_TestLib 子模块。以后改变库位置，只修改 CMake 配置参数；也可直接修改项目 CMake 中的默认路径。示例复制后属于项目，参数不会被库的生成工具覆盖。

## 2. 构建独立 H723 固件

这一步要求匹配 [H7 示例板级前提](../examples/h7_integration/README.md) 的 Cube 工程：H723XG、24 MHz HSE、对应的外设句柄与引脚、TIM2 HAL 时基以及 CMSIS-RTOS2/FreeRTOS。当前验收读取 zzrobot 的 Core、Drivers、Middlewares 和启动汇编，不修改或编译其旧 User/main/freertos 业务。

```powershell
$libraryRoot = 'D:/STM32Projects_HAL/Mrobot_Test_Version/MR_TestLib'
$cubeRoot = 'D:/STM32Projects_HAL/board-H7/zzrobot'
$projectRoot = 'D:/STM32Projects_HAL/my_robot_h7'
if (Test-Path -LiteralPath $projectRoot) { throw '请选择新的项目目录' }
Copy-Item -LiteralPath "$libraryRoot/examples/h7_integration" -Destination $projectRoot -Recurse
cmake -S $projectRoot -B "$projectRoot/build" -G Ninja `
  "-DCMAKE_TOOLCHAIN_FILE=$projectRoot/arm-none-eabi.cmake" `
  "-DMR_LIBRARY_ROOT=$libraryRoot" "-DMR_CUBE_ROOT=$cubeRoot" `
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$projectRoot/build" --parallel 4
arm-none-eabi-nm -u "$projectRoot/build/mr_h7_integration.elf"
```

预期生成 ELF、BIN 和 map，nm -u 无输出。MR_LIBRARY_ROOT 与 MR_CUBE_ROOT 是两个独立依赖，不能互相替代。复制后的项目使用自己的 board/app；CMake 明确引用库 examples/device_execution/app 的控制循环辅助头和 examples/mrlink/board 的 DMA/通信辅助头，通信辅助头还引用 examples/mrlink/app 的遥测类型。这些属于共享示例代码，不是 User 公共 API。若要修改控制循环或通信格式，可将相关辅助头及其依赖复制到自己的 app/board，并显式调整 include 路径；不必修改库。

当前示例不支持任意 Cube 工程只换路径即运行。已有其他硬件项目时，保留自己的启动/链接脚本、时钟和 HAL 初始化，按 [板级接入](BOARD_INTEGRATION.md) 接入实际 board_adapter；不要把本示例的 H723 硬件假设带到其他板卡。

## 3. 从整机切换到独立底盘任务

小型底盘任务复用 H7 的板级入口和底盘装配。将 examples/chassis_task 复制为 H7 项目的相邻 chassis_task，保持以下布局：

```text
workspace/
├─ my_robot_h7/              # 上一步复制的 H7 项目
│  ├─ CMakeLists.txt
│  ├─ board/
│  └─ app/
└─ chassis_task/
   └─ app/
      ├─ chassis_task.cpp
      └─ task_params.hpp
```

在上一步 PowerShell 变量仍有效时：

```powershell
$taskRoot = Join-Path (Split-Path $projectRoot -Parent) 'chassis_task'
if (Test-Path -LiteralPath $taskRoot) { throw '任务目录已存在，请先核对内容' }
Copy-Item -LiteralPath "$libraryRoot/examples/chassis_task" -Destination $taskRoot -Recurse
cmake -S $projectRoot -B "$projectRoot/build-chassis" -G Ninja `
  "-DCMAKE_TOOLCHAIN_FILE=$projectRoot/arm-none-eabi.cmake" `
  "-DMR_LIBRARY_ROOT=$libraryRoot" "-DMR_CUBE_ROOT=$cubeRoot" `
  -DMR_CHASSIS_TASK_EXAMPLE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$projectRoot/build-chassis" --parallel 4
arm-none-eabi-nm -u "$projectRoot/build-chassis/mr_h7_integration.elf"
```

也可直接将任务源放入自己的 app/tasks，并在自己的 CMake 中替换 add_executable 的源列表，不要求长期保留相邻目录。chassis_task.cpp 通过目标的项目 include 路径使用 app/chassis_assembly.hpp 和 board/bindings.hpp；不再通过相对路径猜测 H7 项目目录名。

任务存储为静态栈与控制块；在 osKernelInitialize 后、osKernelStart 前创建。板级端点登记在构造控制对象之前完成；BSP_FDCAN_Init、设备注册和控制初始化在运行中的任务内执行。复制后改 task_params.hpp 即可调整周期/栈/优先级，业务直接改 chassis_task.cpp。

## 4. 项目中具体修改什么

| 需求 | 修改位置 | 接入时需要保持的约定 |
| --- | --- | --- |
| 换硬件总线/管脚 | Cube 外设配置、board/*_endpoints.c、bindings.hpp | HAL 初始化后登记，名称查找不能早于登记 |
| 换时间源 | board/time_source.c 及初始化/IRQ | 单调微秒 epoch，设备和模块共享同一时间基准 |
| 改 PID/几何/限位/方向 | app/robot_params.hpp 及实际设备配置 | 按接口单位填写，方向、零点、减速只换算一次 |
| 换电机/轮子 | app/motor_config.hpp、chassis_assembly.hpp | 传入同一总线端口，核对 ID/命令组所有权 |
| 改机械臂关节数 | Arm<N>、参数数组、arm_assembly.hpp | N 编译期固定，显式增减电机/关节连接 |
| 改遥控/工作模式 | app 业务或任务 | 项目管理命令有效期、故障与重新使能策略 |
| 加库功能 | User 实现/接口、必要 CMake target、最小示例 | 无需维护 YAML、Schema 或生成模板 |

项目启动顺序为：HAL/最终时钟 → 时间源就绪 → 其余外设初始化 → 端点登记与 FDCAN 过滤配置 → 内核初始化/创建任务 → 内核运行 → BSP 启动/设备与模块注册 → 有效反馈 → 显式使能。

H7 示例参数是演示值，默认 kAllowMotorTx=false、运行请求关闭；编译成功不会自动开启电机。机械臂关节/控制对象和应用任务使用固定存储，BSP 的订阅节点、队列等仍有启动期动态分配。

## 5. 验收记录与边界

2026-11-08 已将示例复制到 MR_TestLib checkout 之外的新工作目录，以显式 MR_LIBRARY_ROOT 接入：

- 主机消费项目配置、编译、运行通过；仓库内默认路径也通过。
- H723 整机与独立底盘任务均从新的构建目录编译、链接，输出 ELF/BIN/map，nm -u 无未定义符号。
- 整机 Flash/RAM 为 103256/161408 B；底盘任务为 61244/125888 B，RAM 含现有 heap/MSP 预留。

这些结果验证依赖、接口和链接。未烧录、未连接机器人；栈预算、实时性、DMA/Cache、总线故障恢复及控制方向/标定仍需实际项目验收。当前 H7 时间实现要求最坏 IRQ 延迟小于 1 ms，不支持暂停 TIM2 或低功耗连续计时，见 [时间源](TIME_SOURCE.md)。

Windows 没有 Ninja 时可将生成器换成 `-G "MinGW Makefiles"`，并传入本机 mingw32-make 的 CMAKE_MAKE_PROGRAM；主机构建再按需提供 GCC/G++ 绝对路径。更换生成器或工具链时使用新的 build 目录，不沿用旧缓存。

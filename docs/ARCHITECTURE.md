# 库架构与维护原则

当前设计以 C/C++ 库的易维护性为首要目标。项目通过普通代码完成装配，CMake 处理编译依赖。不要求 Python、YAML 或代码生成器；旧的两套 YAML 配置、Schema、mrctl 与装配/任务生成链路已删除。未来自动配置只能作为独立可选工具，不能成为新增功能的前置工作。

## 分层

```text
项目 app/task → module → robotics / device → component
                                   device → board 适配
```

- component：数学、滤波、控制器、轨迹、容器和协议等通用能力。
- robotics：机构模型和算法，不依赖 HAL、RTOS 或具体电机。
- device：电机、执行器、轮子、关节、传感器；通过传入的总线接口访问硬件。
- module：统一机器人能力接口，管理初始化、命令、反馈、控制及故障。底盘构形由配置选择 robotics 模型，设备由项目传入。
- 项目 board：Cube 初始化、硬件端点、时钟、DMA/Cache、IRQ 和 BSP 实现。
- 项目 app/task：静态对象所有权、装配、参数、任务、遥控与业务状态机。

统一接口服务于手写项目，也为将来的可选自动配置保留清楚的装配边界。不要为尚未实现的工具增加全局注册中心、运行期工厂或平行配置表。

## 固定容量与所有权

机械臂使用 `mr::module::arm::Arm<N>`，N 编译期固定。模型、关节向量、轨迹、输出数组和控制器都使用固定存储；`std::optional<Controller>` 在对象内部原位构造，不分配堆内存。关节、电机和模块由项目静态持有，借用引用/指针的对象必须比使用者活得更久。更改关节数需要重新编译，并显式调整项目的设备连接。

底盘通过 WheelDevice 接口连接具体轮子；机械臂通过 IActuator 连接关节。方向、传动比与零点只在约定层换算一次。模块不自动创建线程、初始化 HAL 或使能电机。

固定容量模块不等于整个固件无堆：当前 H723 BSP 的消息管理、队列等仍使用启动期分配，参考 FreeRTOS 也保留 heap。示例显式静态分配应用线程栈与控制块；更严格的全工程无堆目标需要单独审查。

## 参数和装配

示例 `examples/h7_integration/app/robot_params.hpp` 保存有类型的配置值，`chassis_assembly.hpp` 和 `arm_assembly.hpp` 直接创建并连接设备。它们是项目文件，复制后直接修改。几何/限位/PID 等属于 app 参数，UART/CAN 用途选择属于 board/bindings.hpp，波特率/引脚仍由 Cube 配置。

项目可以拆分参数文件，按功能独立管理；不要求所有项目使用相同的一套巨型配置结构。模块必须提供可直接调用的类型化接口、明确的单位和有效性检查。

## 编译依赖

`cmake/MrTargets.cmake` 直接声明公共 target、源文件、include、编译标准和依赖。不再维护 JSON target 清单或 Schema 枚举。项目通过 add_subdirectory 和 target_link_libraries 选择能力，传递依赖自动跟随；BSP/HAL 由 board target 提供。

## 新增功能的维护工作

| 场景 | 需要维护 | 不要求维护 |
| --- | --- | --- |
| 新算法 | 实现、接口说明、适用验证；有编译源时登记 target | Python、Schema、生成模板 |
| 新设备 | 驱动与既有接口适配、编译依赖、使用示例 | 模块的型号分支、全局工厂 |
| 新构形 | robotics 算法及统一模型选择逻辑、验证 | 复制整套 module 控制入口 |
| 新模块 | 配置/生命周期/控制接口、直接依赖、最小手写示例 | 自动装配生成支持 |
| 新参数 | 配置类型、默认值/有效性检查、使用位置 | 平行 YAML 字段与生成字符串 |
| 项目业务 | 项目 app 的状态机、任务、命令来源 | 共用库的项目专属分支 |

维护者优先让功能直接可用。只有多个项目反复使用且接口稳定的装配，才重新评估可选工具的收益。

## 当前边界

H723 示例有完整构建基线。UART/PWM/SPI/GPIO/FDCAN 由 board 登记，BMI088 显式接收硬件配置；时间通过直接链接的 Board_Time* 实现，TIM2 选择仅在示例 board。BSP 仍使用 H7 HAL，GPIO 的 EXTI 适配仍面向 H7 M7，SPI 当前单所有者，FDCAN 总线操作由项目串行调度。详见 BSP_PORTABILITY_AUDIT.md。本库仍为测试版本，未进行本轮板上验证。

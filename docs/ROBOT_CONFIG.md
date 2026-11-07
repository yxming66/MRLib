# 机器人参数配置 v2

新项目以 [`examples/h7_integration/config.yaml`](../examples/h7_integration/config.yaml) 为完整范本，保持字段含义、层次和单位一致，修改名称、实例、机构和参数即可。文件名统一建议 `config.yaml`，`version: 2` 标识这份完整机器人配置契约。

**当前支持：整份配置校验，以及显式选定的单 CAN 总线 RM 四轮底盘生成。** H7 示例已接入：底盘型号、轮序、轮径、几何尺寸、限速、PID、方向、外部传动比、反馈超时、控制周期和电机发送许可会改变生成结果及固件。机械臂、通信和任务创建仍在 app/board 手写；这些配置目前只校验，不生成。v1 `project.yaml` 的主机流程保持独立。

## 配置分层

| 区域 | 填写内容 | 最终用途 |
| --- | --- | --- |
| `project` | 项目名、库相对路径 | 构建入口与库引用 |
| `board` | 板级适配包、Cube 路径、逻辑总线到 BSP 端点的映射 | 连接已有 board；不在这里重写引脚、时钟、DMA、ISR |
| `runtime` | 是否允许电机发送、是否自动使能 | 显式输出策略；本版要求 `auto_enable: false` |
| `devices.motors` | 型号、总线、地址、方向、传动、超时和协议范围 | 实例化电机及每总线的公共发送管理 |
| `devices.wheels/joints` | 电机引用、轮径、零点、限位 | 连接轮子、关节与物理执行器 |
| `modules` | 统一模块类型、算法模型、设备绑定和控制参数 | 选择 robotics 算法并配置模块 |
| `communications` | 协议、逻辑端口、缓冲、遥测周期和 codec 标识 | 通信装配；消息结构和编解码仍在 app 中 |
| `tasks` | 周期、栈、优先级、入口、所拥有的模块与通信 | 调度装配；任务内部业务逻辑仍在 app 中 |
| `extensions` | 团队命名空间下的自定义业务参数 | 为 app/后续注册扩展保留，公共工具不自动解释 |

未使用的设备类别、通信等填写空映射 `{}`；模块和任务至少各一个。例如仅做底盘的项目可以删除 arm 模块、三个关节及相应电机，并从任务的 `owns_modules` 中移除 arm，保留其余格式。不要每个项目另起一套字段名称。

每个设备的 CAN ID 只在 motors 中写一次；轮子/关节引用电机名，模块引用轮子/关节名，任务引用模块名。v2 不要求同时填写 `project.targets`；当前底盘生成器选择 motor、wheel、module_chassis target，由已有 CMake target 图传递其底层依赖。

## 单位与模型约定

- 统一使用 SI 单位，并在物理量字段名中写出单位：`_m`、`_rad`、`_ms`、`_us`、`_nm`。不使用角度、转速 RPM 或不明含义的 `ratio`。
- `external_ratio` 是电机输出轴转角 / 机构输出转角，只表示额外外部传动。DM/LZ 内部减速已包含在厂商协议中，不再计算一次。关节层不再重复填写同一传动比。
- `zero_offset_rad` 沿用当前 JointT 约定：`q_joint = q_motor - zero_offset_rad`。方向反转在电机层的 `reverse` 设置；不再用负减速比表示反转。
- 底盘 `half_wheelbase_m` / `half_trackwidth_m` 分别是前后、左右轮距的一半，对应当前 C++ 几何结构的 `wheelbase_m` / `trackwidth_m`。显式写 half 避免把全轮距误填进现有算法。
- 四轮顺序固定为右前、左前、左后、右后。车体系 +x 向前、+y 向左、+z 向上，正角速度按右手规则。
- 变换中的平移以父坐标系表达；`T_parent_child` 把子坐标变换到父坐标，`rpy_rad=[roll,pitch,yaw]`，旋转为 `Rz(yaw)*Ry(pitch)*Rx(roll)`。机械臂 `axis_in_joint_frame` 必须是单位向量。
- 机械臂关节列表的条目数决定 `Arm<N>` 的 N，列表顺序决定串联顺序；轴与变换属于算法模型，关节零点与执行器映射属于设备参数，不重复校正同一个偏差。
- 底盘控制器采样周期由所属任务的 `period_us` 推导，不再维护第二个频率字段。生成器限制周期不超过 100 ms；H7 示例额外要求 1000 Hz RTOS 下的整数毫秒周期。任务栈、优先级和入口仍需在 board 中维护。

首版契约覆盖库中两种四轮模型、独立旋转关节的串联机械臂、RM/DM/LZ 电机参数和 UART MRLink。机械臂执行器要求 DM/LZ 原生 MIT 能力，排除 H3510；尚未定义惯性模型，故要求关闭重力补偿。耦合关节、直线关节、更多外设和自定义适配器登记后续通过明确类型扩展，不能用不受检查的任意字段假装已经支持。

## 校验

v2 校验和生成依赖安装在独立虚拟环境中；既有 mrctl v1 和直接链接库 target 仍不需要第三方 Python 包：

```powershell
py -3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r tools/requirements-config.txt
.venv/Scripts/python.exe tools/validate_robot_config.py examples/h7_integration/config.yaml
.venv/Scripts/python.exe -m unittest tools.test_robot_config
.venv/Scripts/python.exe tools/mrctl.py generate examples/h7_integration/config.yaml `
  --module chassis --output-dir build/config-preview/generated
```

校验器只读文件，不生成、复制、覆盖代码，也不调用 CMake。格式规则由 [`robot.schema.json`](robot.schema.json) 描述：未知公共字段报错，重复键报错，类型、单位字段和枚举保持一致。示例第一行的 yaml-language-server 注释可让支持 JSON Schema 的 YAML 编辑器使用该规则；复制到其他项目时需调整 Schema 相对路径或在编辑器中关联它。

另外检查：有限数值、总线/设备/模块引用、旋转轴、限位、共享电机/轮子/关节、任务唯一所有权、UART 独占以及 CAN 端点冲突。RM 的分组发送地址也计入检查，DM 的 `can_id+0x100/+0x200` 保留地址不能只因当前选择 MIT 模式就忽略；LZ 同一总线的同一个电机目标不允许通过换 host 重复占用。

独立校验不验证外部路径的存在，方便跨开发者提交可移植配置。生成时要求 `project.library` 指向运行工具所属的库 checkout；CMake 接入时进一步核对 board profile 和所选 CAN 端点能力。任务 entry、codec、资源容量、实时性和机械参数仍需项目实现与检查。此阶段没有执行板上验证。

## 生成边界与接入

目标结构与流程保持为：

```text
robot_project/
├── config.yaml
├── board/                  # 手写/Cube 板级代码
├── app/                    # 手写业务、任务逻辑、消息 codec
├── build/generated/        # CMake 默认生成位置，CLI 也可显式选择 generated/
└── external/MR_TestLib/    # 库可用 submodule 固定版本
```

配置校验 → 类型/依赖与板级能力检查 → 生成静态 C++ 装配和构建清单 → 编译。YAML 只作为电脑上的构建输入，不在 MCU 上解析。不要求勾选每个底层源文件，不从配置猜测任意 `.h/.c`，也不在 YAML 中放入 shell 命令或 C++ 片段。

输出为 `robot_config.hpp`、`robot.cmake` 和 `.mrctl-manifest.json`。`ChassisAssembly` 拥有一组 RM 命令总线、四电机、四轮与统一 Chassis；构造时注入项目的 CanPort，显式调用 `Register()`，再由 app 调度反馈、控制与输出。注册不自动 Enable，发送许可常量需要 app 的端口适配执行（H7 已执行），不能把它当作通用 CanPort 的隐式限制。

生成器会校验整份 YAML，但只生成 `--module` 选中的底盘，拒绝其他模块类型、非 RM 轮电机及跨 CAN 总线装配。Arm/DM/LZ、MRLink、任务模板、自定义外设注册尚未生成；选定底盘以外的对象与依赖由 app 继续维护。`board.cube_root` 不会隐式加载或扫描外部工程，实际 Cube 输入由项目 CMake 显式指定。

重新生成只更新名为 `generated` 的目录中的固定产物，不覆盖 `app/`、`board/` 和原始配置。摘要清单检查已有产物归属及内容，遇到手改或没有清单的同名文件会失败，且在检查完成前不写入；内容不变保留时间戳。单个文件采用原子替换，整组产物不是跨文件事务，请勿并发写同一输出目录。中断后若摘要不匹配，应保留需要的手改，再删除专属 generated 目录并重新生成。输出路径不接受符号链接/目录联接。

项目通过 `mr_target_robot_config()` 接入，修改 YAML 后 `cmake --build` 自动重新配置和编译。板级注册、BSP 源依赖和完整接入步骤见 [板级接入](BOARD_INTEGRATION.md) 与 [H7 示例](../examples/h7_integration/README.md)。

格式破坏性变化必须升级 version，并提供迁移说明。标定结果初期可直接写回对应参数；在线标定和 Flash 覆盖机制尚未实现。

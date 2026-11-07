# 通用机械臂模块

`mr::module::arm::Arm<N>` 连接 robotics 算法和项目提供的 `IActuator`。
`N` 在编译期确定且运行中不变；模型、控制器参数和每关节增益由 `Config` 传入并复制。关节向量、轨迹、输出和控制器使用对象内部固定存储，不动态分配关节或通过堆创建控制器；optional 只做原位构造。
执行器由项目持有，生命周期必须覆盖模块；不创建电机，不初始化总线。

```cpp
using Arm = mr::module::arm::Arm<3>;
Arm arm;
Arm::Config config;
config.model = project_model;
config.output.actuators = {&joint1, &joint2, &joint3};
config.output.gains = {{{20, 1, 1}, {15, 1, 1}, {10, 0.5f, 1}}};
// 检查下面每一步的返回值；先由 board 初始化硬件。
arm.Init(config);          // 校验并注册，保持未使能
arm.UpdateFeedback();
arm.Enable();             // 必须有有效反馈，默认保持当前位置
// 每个调度周期：
arm.UpdateFeedback();
arm.SetCommand(command);  // robotics::arm::ArmCommand<3>
arm.Step(0.002f);
arm.CommitOutput();       // 每次成功 Step 最多提交一次
```

- `Config` 拒绝空指针、同一执行器重复绑定、无效模型/增益。不同对象的总线地址冲突由设备/项目注册管理。
- 执行器接口必须使用关节坐标：转动关节 rad、rad/s、Nm；移动关节 m、m/s、N。减速器、方向、零偏和耦合转换在 device/项目适配器完成；不能直接绑定尚未换算的电机坐标。
- 每次 Step 消费一次成功 UpdateFeedback 的快照；此调用约束不替代设备层真实帧龄/超时检查。
- 反馈、使能、算法、控制准备、发送失败进入 Fault，清除全部待发控制并尝试 Relax；主错误与 Relax 错误分别可查询。发送不是总线原子事务，已发送帧不能撤回。
- 修复故障后先 UpdateFeedback，再显式 Enable。恢复重置轨迹及目标，不自动续跑旧命令。注册失败不能在同一实例恢复：当前设备契约没有注销接口，需项目重新装配。
- Disable 后才能 SetZero；设零使反馈失效，必须重新采样。析构不访问硬件，应用负责在停止调度后 Disable。
- `output()` 保留算法状态、是否采用位置 IK 回退等诊断；设备温度等具体诊断从项目持有的设备读取。

三轴几何、质量、工作空间与遥控映射位于 `examples/three_axis_arm/app`，不参与库目标的编译。
旧 `mr::arm::Runtime`、`Arm_Params_t` 和具体电机访问接口已从公共库删除，不提供静默兼容路径。

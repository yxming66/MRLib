# CmdCenter：DR16 / NUC 到模块命令

工程策略集中在 `cmd_center.cpp`，复用库的 `component/cmd_center/cmd_center.hpp`。共享消息放在 `app_messages.hpp`；输入适配器、条件、Builder、Sink 和 Center 实例均为 cpp 私有类型。

## 阅读顺序和执行链

先读 `app_messages.hpp` 的类型，再读 `cmd_center.hpp` 的绑定接口，最后按 `cmd_center.cpp` 的编号注释阅读。`chassis_task.cpp` 展示实际调用，`tests/cmd_center_host_tests.cpp` 提供可编译的输入注入示例。

```text
DR16 接收任务 → RemoteInput 快照 → Dr16Input.update
                                  ├─ 新鲜度 / 布防 / 使能
                                  └─ select_chassis_mode 回调
NUC 接收任务  → NucInputSnapshot → NucInput.update
                                      ↓
          global().safety → 路由 when → Builder → 仲裁
                                      ↓
                       Sink → ChassisCommand → 模块任务
```

NUC 协议和接收任务需由项目实现；模板默认不绑定 read_nuc。CmdCenter 不收串口、发送 CAN 或运行 PID。底盘任务只读反馈、保存命令，control_enabled 固定为 false。

## 显式绑定模式回调

`ChassisApplication::Init()` 中：

```cpp
const app::CommandCenterBindings bindings{
    nullptr, ReadRemote, nullptr, app::SelectChassisMode};
if (!app::InitCommandCenter(bindings)) return false;
```

第三项是 NUC 读取函数，第四项是模式回调。Dr16Input 先检查遥控有效性和使能，再调用回调；适配器中不写拨杆映射后备逻辑。未绑定模式回调时保持 Disabled。

| 右拨杆 | 期望模式 | 实际输入 |
| --- | --- | --- |
| 中 | Manual；左拨杆下档时 StairAssist | DR16 |
| 下 | Auto | NUC |
| 上 / 未知 | Disabled | 安全命令 |

左拨杆另管全局授权：启动后先置上档布防，上档期间停止；再拨中/下档才允许控制。遥控失联清除布防，恢复后必须重新经过上档。自动模式也受此约束。

## 授权、优先级与仲裁

`when` 决定候选是否有资格参加，`priority` 决定有效候选先后，`arbitrate` 决定选择和同级冲突策略。不能只给 NUC 更高优先级，否则它一在线就可能抢走手动控制。

本例手动/自动的 when 根据模式互斥，使用 `HighestPriority{TieBreak::RejectConflict}`。以后若增加同时有效的路由，高优先级胜出，同级冲突走安全输出。全局 safety 优先于这些路由。

输入适配器检查在线、原始时间戳和有限数值；online=false 时该输入不能产生候选。Auto 下 NUC 超时或未绑定读取回调，期望模式仍为 Auto，实际来源变成 None，输出禁用零命令，不自动回退手动。

## 映射和时间戳

DR16 死区 0.05。普通手动：右 Y → vx，右 X 取反 → vy，左 X 取反 → wz；最大值为 1.5 m/s、1.5 m/s、4 rad/s。符号须按实际坐标校验。

StairAssist 展示同一输入在另一模式下映射到不同字段：左 Y → 前机构角度目标变化率，左 X → 支撑机构角度目标变化率，右 Y → 两侧履带速度；wz 为零。模板未装配这些机构，不需要时可同时删除对应模式、字段和 Builder 分支。

NUC 的 vx/vy/wz 限幅到 robot_config.hpp 的自动速度上限，不授权台阶机构。每个 Builder 生成完整命令，不依赖上一周期字段；安全命令清零全部运动字段、撤销 enabled。

received_ms 是命令生成时间，source_received_ms 是原始输入在本板的接收时间。重复 tick 只更新前者，不能掩盖输入超时。NUC 不可直接填写未同步的 PC 时间。

## 绑定 NUC

接收任务解析协议后保存最新 NucInputSnapshot，填写本板接收时间、sequence、valid。读写须用短临界区等同步方式保护，读取回调快速复制、不阻塞，不得在读取时刷新接收时间。

```cpp
bool ReadNuc(void* context, app::NucInputSnapshot& out) {
  return static_cast<BoardInputs*>(context)->ReadNucSnapshot(out);
}
const app::CommandCenterBindings bindings{
    &board_inputs, ReadRemote, ReadNuc, app::SelectChassisMode};
```

BoardInputs 和 board_inputs 由项目定义，生命周期必须覆盖 Center。所有回调共用 context，可以使用或忽略它。测试文件中的 Inputs/ReadNuc 是可直接编译的注入例子。

## 为云台独立选源

在共享消息中增加 GimbalCommand 和云台期望模式，在 cpp 注册第二个 output，为它配置独立的条件、Builder、safe 和 Sink。增加 select_gimbal_mode 回调，可以同样读拨杆，也可以读按键状态，不要用 chassis_mode 控制全部模块。

同一输入可供多个输出使用，各输出独立仲裁，例如底盘手动、云台 NUC 自瞄。全局 safety 作用于所有输出，模块专属限制放在其路由条件。同步调整 `Center<输入数, 输出数, 路由槽数>`；槽位还包含安全规则和 safe Builder，须检查 overflowed。

## 线程和诊断

同一任务独占 Init/Update/GetStatus。相同绑定重复初始化幂等，不支持运行中更换回调。输入接收任务只写自己的快照，不直接操作 Center。

本模板 Center 和底盘反馈同属一个任务，Sink 直接保存值，Update 按值返回，无需中间命令队列。其他任务通过 ReadChassisStatus 读取同周期的命令、Center 状态和反馈；它与 RemoteControlTask::ReadLatest 都只能在运行中的任务上下文调用。

多个模块分属不同任务时，选一个 Center 所有者，各 Sink 向模块任务发布受同步保护的最新命令；消费者还要检查命令年龄，以应对中心停止更新。不要让多个任务同时 tick 同一实例。

观察 chassis_mode（期望）、chassis_source（实际）、remote_fresh/nuc_fresh/armed 和 safety_reason。实际来源从最终输出记录，不能从某个 Builder 曾被调用推断。

## 独立主机验证

在 MRLib 根目录用主机 C++ 编译器执行，不需要 Cube/HAL/RTOS：

```sh
cmake -S examples/project_template/tests -B build/cmd-center-example -G Ninja
cmake --build build/cmd-center-example
ctest --test-dir build/cmd-center-example --output-on-failure
```

四组测试直接编译模板的 cmd_center.cpp，覆盖手动映射、台阶模式、死区、布防/重新布防、拨杆切源、NUC 限幅/失效/超时、遥控全局停止、缺少回调和毫秒回绕；自定义模式回调测试确认实际选择由回调决定。Windows 可用 -DCMAKE_CXX_COMPILER 指定主机 G++；不要用 ARM 裸机编译器运行主机测试。

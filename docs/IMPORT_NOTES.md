# MRobot 测试版本

整理日期：2026-10-26。

## 目录

- `User/`：初次整理为 283 个文件；后续按 Git 提交增删。以 RC_02 的当前磁盘内容为基础，再合入 zzrobot 的 10 个差异文件。
- `_reference/User_code/`：旧版可信代码快照。原始 156 项中有 3 个 macOS `.DS_Store` 元数据文件；清理后保留 153 个可复用源文件、文档和配置，不自动加入当前测试代码编译。
- `SOURCE_MANIFEST.csv`：初次导入版本的逐文件来源和 SHA-256 历史记录；后续改动以 Git 提交为准。
- `BASELINE_MANIFEST.csv`：清理后的 153 个旧版参考文件 SHA-256。
- `build/`：仅用于 CMake 和主机测试的本地生成目录；由 `.gitignore` 排除，可按构建说明重建。

## 来源与选择规则

旧版可信基线：`D:\apps\MRobot\MRobot\assets\User_code`

后续开发主体：`D:\STM32Projects_HAL\board-H7\RC_02\User`

机械臂后续分支：`D:\STM32Projects_HAL\board-H7\zzrobot\User`

RC_02 和 zzrobot 各有 283 个文件，路径集合完全相同，273 个文件内容完全相同。仅以下 10 个文件不同，全部采用 zzrobot 内容：

- `module/arm/arm_command_adapter.cpp`
- `module/arm/arm_model_config.cpp`
- `robotics/arm/control/arm_controller.hpp`
- `robotics/arm/core/command.hpp`
- `robotics/arm/core/status.hpp`
- `robotics/arm/kinematics/inverse_kinematics.hpp`
- `robotics/arm/README.md`
- `robotics/arm/tests/arm_host_tests.cpp`
- `robotics/core/rotation.cpp`
- `robotics/core/rotation.hpp`

最后两个文件是机械臂算法使用的共享旋转数学依赖，因此一并同步。初次导入时，User 内容与 zzrobot/User 完全一致；分文件来源记录区分 RC_02 主体与 zzrobot 更新。之后的变更由 Git 提交记录维护。

旧版采用按库分子目录的布局，RC_02 已将大量 C 文件移到各层根目录，并新增 C++ 库。为避免重复实现和旧配置混入编译，没有把旧目录直接叠加到 User。需要旧版独有驱动时，可从参考快照单独迁移。参考快照是历史可信版本；新增内容仅表示已同步并通过下述检查，不表示全部完成硬件验证。

## 验证

- 初次导入的 283 个用户代码文件均通过 SHA-256 核对。
- 清理后的旧版参考快照包含 153 个内容文件，基线 SHA-256 可按 `BASELINE_MANIFEST.csv` 复核。
- 已运行原有 `User/robotics/arm/tests/run_host_tests.ps1`，结果：`arm host tests passed`。
- 初次导入时未修改三个源目录。
- 本目录是 User 代码测试集合，不是完整 STM32 工程；未复制 Core、Drivers、Middlewares、链接脚本和板级构建配置，未进行完整固件编译、烧录或实机测试。

## 2026-11-05 整理

- 从参考快照中删除 macOS Finder 生成且 Git 已忽略的 `.DS_Store`；基线清单只保留可复用内容。
- 从 `User` 删除 1 字节临时文件 `module/autoCtrlAPI/ore_store/auto_ore_store.c.new`。
- 删除未加入任何工程、整份被注释掉的 RM M2006 限位调试任务 `device/motor/packages/soft_limit_learning/soft_limit_learning_test`。
- 清除本地 `build/` 和 Python 字节码缓存。目录结构按原有 BSP、component、device、module、robotics、task 层次保留；本次不移动公共头文件或源码，避免破坏包含路径和板级工程引用。

在安装了 g++ 的 PowerShell 中，于本目录执行以下命令可重跑已有主机测试：

```powershell
.\User\robotics\arm\tests\run_host_tests.ps1
```

# robotics/arm

`User/robotics/arm` is the hardware-independent, fixed-size arm algorithm
library. It is intentionally small enough to read from top to bottom and does
not include device, module, BSP, HAL, CAN, or RTOS headers.

## Public Entry

Include `robotics/arm/arm.hpp` for the complete public API. The library is
organized as:

- `core/`: arm types, commands, and status values.
- `model/`: fixed-size joint and arm geometry, limits, and inertial data.
- `kinematics/`: forward kinematics, geometric Jacobian, pose error, and one
  bounded numerical LM inverse-kinematics solver.
- `planning/`: synchronized joint trapezoid planning and Cartesian cubic-blend
  planning.
- `safety/`: explicit joint and command checks and position clamping.
- `dynamics/`: static gravity compensation and recursive Newton-Euler inverse
  dynamics for motion feedforward.
- `control/`: the single high-level `ArmController<DoF>` composition point.
- `tests/`: host-only regression tests with no firmware dependencies.

The normal control flow is:

```cpp
ArmModel<3> model = MakeModel();
ArmController<3> controller(model, config);
controller.Reset(feedback);
const ArmOutput<3> output = controller.Update(feedback, command, dt_s);
```

`ArmController` owns fixed-size planner state and returns joint-space output.
It never knows how many motors implement a joint. Motor direction, gear ratio,
zero calibration, and multi-motor coupling belong to `device/joint` or the
module output adapter.

When `ArmControllerConfig::enable_position_only_fallback` is explicitly enabled
(default: false), a requested full-pose Cartesian IK solve that ends with
`kNoConvergence` or `kSingularity` is retried once with orientation rows
disabled. This policy lives in `ArmController`, not in the numerical IK
solver. The returned `ArmOutput::position_only_fallback` flag identifies the
degraded cycle. `primary_ik_status` preserves the initial solve status; successful
fallback still reports kOk with position_only_fallback=true. Applications that
require orientation should leave fallback disabled.

The IK result also reports `minimum_singular_value`. Below
`singularity_sigma_safe`, damping is increased continuously up to
`maximum_damping` and `near_singularity` is set. This limits the large
`j4`/`j6` counter-rotation typical of a wrist singularity, but cannot restore
an orientation degree of freedom that is physically lost. A Cartesian planner
should slow down or reroute before the stop threshold; the controller can use
the existing position-only fallback when full-pose convergence is unreliable.

Dynamic feedforward is also a single call:

```cpp
const auto compensation = dynamics::ComputeDynamicCompensation(
    model, feedback.position, feedback.velocity, command.acceleration);
```

It returns the combined gravity, inertia, Coriolis, and centrifugal effort.
Each link model stores mass, center of mass, and a symmetric positive
semidefinite inertia tensor expressed at the center of mass in the link frame.
An all-zero inertia tensor remains valid for point-mass models.

Static Cartesian goals use `CartesianPlanner` followed by numerical IK.
Continuously changing Cartesian targets, such as a remote-control joystick,
set `ArmCommand::streaming_cartesian`. IK generates a joint target; each joint
tracks it with bounded velocity and per-step velocity change (acceleration).
Output acceleration is the actual change of the generated velocity divided by dt.
Reset uses feedback velocity as the initial reference. The IK maximum joint step
also limits per-cycle travel. Reversing an infeasible target may require overshoot;
a position-limit conflict returns failure instead of silently clamping this path.

Failure/hold/disable are explicit safety transitions and may request immediate
zero velocity. The normal streaming acceleration contract does not promise a
physical emergency braking profile. Non-streaming joint/Cartesian replans remain
rest-to-rest plans: request a mode/goal change from a stopped state or implement
application braking first. Cartesian interpolation limits Cartesian motion; it
does not independently guarantee joint-space derivatives after numerical IK.

All Cartesian targets, FK poses, pose errors, tool-origin geometric velocities,
Jacobian rows, and gravity are expressed in the model world frame. BaseTransform
is T_world_base and ToolTransform is T_last_tool. Joint coordinates are rad for
revolute joints and m for prismatic joints. IK task_weights scale world position
rows (m) and orientation rows (rad) before least squares; choose weights and the
mixed joint max_joint_step according to the robot's units, not as dimensionless
interchangeable physical limits. The model is borrowed and must outlive the
controller without mutation; snapshots, IK scratch data and planners use fixed
storage. Configure and update run in one execution context.

## Host Test

From the repository root:

```powershell
./User/robotics/arm/tests/run_host_tests.ps1
```

The test covers model validation, FK, Jacobian, pose error, numerical IK,
joint and Cartesian planning, safety, static and dynamic compensation, control,
and continuously changing Cartesian targets.

## Firmware Boundary

`User/module/arm` is a generic Arm<N> capability and output adapter. The app owns
model dimensions, joint objects, command mappings and scheduling. No project
hardware is instantiated by the algorithm library or module. The algorithm
library does not parse URDF/DH at runtime or contain analytic solver registries.

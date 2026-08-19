# FrankaWeightLift

A ROS 1 Noetic (Ubuntu 20.04) catkin package for a Franka Research 3 (FR3) that renders a
**constant downward force at the end effector** while leaving the end effector **free to be
lifted and moved by hand inside a vertical plane**. A human can hold the tool and lift, lower or
slide it forward and back; the arm keeps pulling towards the earth with the force you asked for and
compensates every other disturbance.

## What the controller does

`franka_weight_lift/WeightLiftController` is a `franka_hw` effort controller plugin (spawned by
`controller_manager`, not a standalone node). The task frame is the **robot base frame O**
(X forward, Y left, Z up). Of the six end-effector DOF:

| DOF | Behaviour |
| --- | --- |
| base **X**, base **Z** | **free** — no position servo. Light damping, a speed cap and virtual walls only. This is the plane the human works in. |
| base **Y** | held by a stiff spring/damper at its startup value, so motion stays in the X-Z plane. |
| all 3 rotations | held by a stiff spring/damper at the attitude measured when the controller started, so the end-effector frame keeps a fixed attitude relative to the robot base. |

On top of that the controller commands `desired_force` newtons along **-Z of the base**, i.e.
towards the earth along gravity.

The commanded wrench (base frame) is

```
w  =  [0, 0, -F]                        commanded pull towards the earth
   +  [0, 0,  k_p*e + k_i*∫e]           optional PI on the transmitted force (gains 0 by default)
   +  initial_wrench                    cancels the wrench already on the flange at startup
   +  [0, -K_y*(y-y0) - D_y*vy, 0]      base Y held: motion stays in the X-Z plane
   +  [0, 0, 0, -K_r*e_r - D_r*ω]       orientation held at the startup attitude
   +  [-D_f*vx, 0, -D_f*vz]             light damping inside the free plane
   +  wall(x) , wall(z)                 virtual walls, free inside the box
   +  brake(vx, vz)                     braking above max_free_speed

tau = J^T w + nullspace(q0) + coriolis   , then rate- and magnitude-saturated
```

### Design decisions worth knowing

- **Orientation is frozen at startup, not driven to base-axis alignment.** The FR3 flange at a
  downward-pointing pose is about `diag(1,-1,-1)` relative to the base, so commanding literal axis
  alignment would rotate the arm the moment the controller is spawned. Freezing instead means no
  motion at startup while still keeping the end-effector frame fixed relative to the base.
- **The pull is along base -Z, not along the end effector's own Z axis.** They coincide when the
  tool points straight down, and using the base axis guarantees the force is exactly vertical even
  if the startup pose is slightly tilted.
- **Sign convention.** libfranka's `O_F_ext_hat_K` is the wrench the robot applies *to* its
  environment, expressed in base coordinates (see `franka/robot_state.h`), so pulling down is a
  negative z entry both when commanding and when measuring.
- **The startup wrench is measured and cancelled.** An unmodelled tool weight would otherwise add
  to the commanded pull, since the free plane has no vertical stiffness to hold it. The controller
  spends its `INITIAL` state holding the startup pose stiffly in all six DOF until the estimate
  settles, then latches it. **Do not touch the robot until `weight lift active` is logged.**
  Configuring `setLoad` properly is still the better fix; set `compensate_initial_wrench: false` if
  you would rather not have the bias term at all.
- **Force feedback defaults to off.** `k_p = k_i = 0` means pure feedforward `J^T w`, which cannot
  go unstable. Raise `k_p` in steps of ~0.1 with `rqt_reconfigure` to compensate joint friction. The
  integrator is frozen above `integral_freeze_speed` so it cannot wind up while the tool is falling
  with nothing holding it.
- **Safety.** The free plane has no vertical stiffness, so if the human lets go the arm keeps
  pulling down. Three things bound that: virtual walls around the startup pose (`wall_z_min` etc.),
  a speed cap (`max_free_speed`) and per-joint torque saturation. The lower z wall has to hold the
  commanded pull, so it sags by `desired_force / wall_stiffness` (5 mm at 10 N / 2000 N/m).
- **Startup ramp.** `desired_force_` starts at 0 N and is low-pass filtered towards the target
  (`force_filter_gain: 0.001` ≈ 1 s time constant at 1 kHz), so neither spawning the controller nor
  a live force change produces a jolt.

## Build / run

This repo is **not** built in place. `compile_package.sh` wipes `~/workspace/catkin_ws_fwl/`,
copies `franka_weight_lift/` into it and `catkin build`s there against the prerequisite
`catkin_ws_franka` workspace and `~/workspace/libfranka/build`. Edits here do not reach the running
binary until you re-run the compile script.

```bash
# Build the package (from repo root)
./scripts/compile_package.sh

# Run: 10 N downwards, default robot ip
./scripts/run_experiment.sh
# 25 N downwards, explicit robot ip
./scripts/run_experiment.sh 25.0 192.168.1.11
```

Or directly:

```bash
roslaunch franka_weight_lift weight_lift.launch desired_force:=25.0 rqt_reconfigure:=true
```

`desired_force`, `k_p` and `k_i` are live-tunable through `dynamic_reconfigure`
(`rqt_reconfigure` → `weight_lift_controller`). Everything else lives in
`franka_weight_lift/config/weight_lift_controller.yaml`, which is the single source of truth —
`init()` seeds the reconfigure server from it, so the yaml wins at startup.

Before running: release the FR3 brakes and put the robot into `FCI`/execution mode in Desk, as for
any `franka_ros` torque controller.

Code style: Google C++, `IndentWidth: 2`, `ColumnLimit: 100` (`.clang-format`). C++17.

## Layout

```
franka_weight_lift/
  CMakeLists.txt  package.xml
  franka_controllers_plugin.xml            plugin export for controller_manager
  cfg/weight_lift_param.cfg                dynamic_reconfigure: desired_force, k_p, k_i
  config/weight_lift_controller.yaml       all controller parameters
  config/rosconsole.config                 log levels
  include/franka_weight_lift/
    weight_lift_controller.h
    pseudo_inversion.h                     vendored nullspace helper
  launch/weight_lift.launch                franka_control + controller spawner
  src/weight_lift_controller.cpp
scripts/
  compile_package.sh                       -> ~/workspace/catkin_ws_fwl
  run_experiment.sh
```

## Tuning notes

| Symptom | Knob |
| --- | --- |
| tool drifts sideways out of the plane | raise `lateral_stiffness` |
| tool twists in the hand | raise `rotational_stiffness` |
| free plane feels sticky / heavy to move | lower `free_damping` |
| free plane oscillates when released | raise `free_damping` |
| elbow wanders while dragging | raise `nullspace_stiffness` |
| felt force is less than commanded | raise `k_p` (friction compensation) |
| working volume too small | widen `wall_*`; keep `*_min <= 0 <= *_max` |

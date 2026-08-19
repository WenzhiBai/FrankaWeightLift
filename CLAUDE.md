# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`franka_weight_lift` is a ROS 1 Noetic (Ubuntu 20.04) catkin package with a single `franka_hw`
effort controller for a Franka Research 3 (FR3). It renders a constant downward force at the end
effector while leaving the end effector free to be moved by hand inside the vertical X-Z plane of
the robot base. Nothing but that force acts in the plane: base X and Z have no stiffness and no
damping. See `README.md` for the control law, the design decisions and a tuning table.

## Build / run

This repo is **not** built in place. `compile_package.sh` wipes `~/workspace/catkin_ws_fwl/`, copies
`franka_weight_lift/` into it and `catkin build`s there against `catkin_ws_franka` (which supplies
`franka_ros`) and `~/workspace/libfranka/build`. Edits here do not reach the running binary until
you re-run the compile script.

```bash
./scripts/compile_package.sh                     # build; must be run from the repo root
./scripts/run_experiment.sh [force_N] [robot_ip] # real robot, default 0.0 N / 192.168.1.11

# equivalent direct launch (sources must already be in the shell)
roslaunch franka_weight_lift weight_lift.launch \
  desired_force:=25.0 robot_ip:=192.168.1.11 rqt_reconfigure:=true
```

`desired_force` defaults to 0 N and `run_experiment.sh` always opens `rqt_reconfigure`: the
controller is meant to be driven from there, not from the command line. The arm **moves on its own**
when the controller is spawned (it aligns itself with the base), so anything that changes `ALIGN`
needs the same care as the 1 kHz path.

`run_experiment.sh` sources `/opt/ros/noetic`, `catkin_ws_franka` and `catkin_ws_fwl` itself;
`compile_package.sh` appends the `catkin_ws_fwl` source line to `~/.bashrc` / `~/.zshrc`. Before any
run the FR3 brakes must be released and the robot put into FCI/execution mode in Desk, as for any
`franka_ros` torque controller. Ctrl-C in `run_experiment.sh` also strips ANSI escapes from
`~/.ros/log/latest/*.log`.

**There is no test suite, no simulation target and no CI.** Verification is: `compile_package.sh`
builds clean, then a real-robot run reaches `weight lift active` and the throttled telemetry line
shows the expected force and offsets. Treat any change to `update()` as untestable off-robot and say
so rather than claiming it is verified.

Code style: Google C++, `IndentWidth: 2`, `ColumnLimit: 100` (`.clang-format`). C++17.

## Architecture

One `controller_interface::MultiInterfaceController` plugin, registered through
`franka_controllers_plugin.xml` and spawned by `controller_manager` — **not** a standalone node, so
there is no `main()` and no ROS topics of its own (`dynamic_reconfigure` is the only live interface).
Everything is in `src/weight_lift_controller.cpp` +
`include/franka_weight_lift/weight_lift_controller.h`, wired up by `launch/weight_lift.launch`
(which also includes `franka_control.launch`).

Control flow in `update()`: read `franka::RobotState` → build a 6x1 desired wrench in the **robot
base frame** → `tau = J^T * wrench + nullspace + coriolis` → `saturateTorqueRate` → `setCommand`.

The FSM is `INITIAL` → `ALIGN` → `LIFT`, plus a terminal `FAULT`:

- `INITIAL` holds the startup pose stiffly in all six DOF (`poseHoldWrench`) until the external
  wrench estimate settles, then latches it as `initial_wrench_O_`.
- `ALIGN` uses the same stiff hold, but `beginAlignment()` has swapped the hold targets for the
  base-aligned ones (`hold_position_(y) = 0`, `orientation_d_ = diag(1,-1,-1)`), so the arm moves
  itself onto them. It ends when the arm has stopped inside the align tolerances, or on
  `max_align_sec`. `beginAlignment()` returns `FAULT` instead if the startup pose is further off
  than `max_align_y_offset` / `max_align_angle`.
- `LIFT` is the weight-lift law (`liftWrench`). `q_d_nullspace_` is re-latched on entry, since the
  align move changed the joint configuration.
- `FAULT` holds the startup pose and logs; the operator has to reposition and respawn.

### Parameters

`config/weight_lift_controller.yaml` is the single source of truth, read in `readParameters()`,
which also validates the wall bounds and warns about lower-wall sag. `desired_force`, `k_p` and
`k_i` are additionally exposed through `dynamic_reconfigure` (`cfg/weight_lift_param.cfg`); `init()`
seeds the reconfigure server's namespace from the yaml values so the first reconfigure callback
cannot clobber them with the `.cfg` defaults. The launch file's `desired_force` arg wins over the
yaml only because its `<rosparam param=...>` line comes **after** the `<rosparam command="load">` —
keep that ordering.

Adding a parameter means touching every layer, or it silently keeps its header default:
yaml entry (with a comment and units) → member + default in the header → `node_handle.param(...)`
in `readParameters()`. If it should also be live-tunable: `cfg/weight_lift_param.cfg` →
`weightLiftParamCallback()` → the `setParam` seeding block in `init()`.

`CMakeLists.txt` globs `src/*.cpp`, so new sources need no CMake edit but do need a re-run of the
compile script; the `${PROJECT_NAME}_gencfg` dependency is what generates
`weight_lift_paramConfig.h` from the `.cfg`.

### Logging

`config/rosconsole.config` mutes everything at WARN except `ros.franka_weight_lift` at INFO. The
`<env name="ROSCONSOLE_*">` lines in the launch file must stay **before** the `franka_control`
include so the `controller_manager` process inherits them. The only telemetry is one
`ROS_INFO_STREAM_THROTTLE(0.5, ...)` line in `update()` — extend that rather than adding per-tick
logging or a publisher to the 1 kHz path.

## Conventions

- **Base frame is the task frame.** Free axes are base X (index 0) and base Z (index 2); base Y
  (index 1) is held at 0 and all three rotations at `diag(1,-1,-1)`. `getZeroJacobian` is expressed
  in the base frame, so every wrench assembled in `liftWrench` / `poseHoldWrench` must be too.
- **The free plane carries nothing but the commanded force.** No stiffness, no damping. The only two
  terms that ever act in base X/Z are the virtual walls and the `max_free_speed` brake, and both are
  exactly zero until they trigger. Do not add a damping or centering term there — that is the whole
  point of the controller.
- **Wrench signs follow libfranka.** `O_F_ext_hat_K` is the wrench the robot applies *to* the
  environment, in base coordinates, so a downward pull is a negative z entry both when commanding
  and when measuring. Do not "fix" the sign of the `initial_wrench_O_` term: it matches the
  convention documented in `franka/robot_state.h` and used by the sibling `~/repos/Seeing-Is-Enough`.
- **The hold targets change once, in `beginAlignment()`.** `starting()` captures the measured
  startup pose so `INITIAL` stays put; `beginAlignment()` then replaces the Y and orientation
  targets with the base-aligned ones. Setting the aligned targets in `starting()` instead would make
  the arm move while the wrench estimate is still settling and poison `initial_wrench_O_`.
- **Hold stiffness terms are clamped, damping terms are not.** `lateralHoldForce()` and
  `orientationHoldWrench()` clamp only the spring term (`max_lateral_force`,
  `max_rotational_torque`), leaving damping outside the clamp. That is what keeps the align snap from
  slamming the arm across a large offset while still decelerating it on arrival — clamping the sum
  instead would cancel the damping exactly when it is needed.
- **`target_force_` vs `desired_force_`.** `target_force_` is what the yaml/reconfigure asks for;
  `desired_force_` is the low-pass-filtered value actually rendered, starting from 0 at `starting()`.
  Command from `desired_force_` and set only `target_force_` from the callback — swapping them
  removes the ramp and produces a jolt at spawn and on every live change.
- **Walls are armed, not absolute.** `wall_origin_` is latched in `liftWrench` on the
  `target_force_` 0 → non-zero transition and released again once the force is back to 0 *and*
  `desired_force_` has decayed below `kForceOffThreshold` — never on `target_force_` alone, or the
  arm would be left unbounded while the ramp is still pulling it down.
- Anything touching the real robot at 1 kHz: keep it allocation-light, keep the torque saturation,
  and default new force-feedback gains to zero.

## Safety

The free plane has no vertical stiffness and no damping, so if the human lets go while a force is
being rendered the arm keeps pulling down. Virtual walls (anchored at the pose where the force was
applied), the free-plane speed cap (`max_free_speed` / `brake_damping`) and per-joint rate +
magnitude torque saturation (`delta_tau_max_`, `tau_max_`) are the only things bounding that — do not
remove them, and keep `wall_*_min <= 0 <= wall_*_max` so the walls never arm around a pose that is
already outside them.

While `desired_force` is 0 the walls are off by design and the plane is unbounded. That is deliberate
(it is how the tool gets carried to the work), but it means the only bounds during that phase are the
speed cap, the torque limits and the joint limits.

Two operator-facing facts that must stay in the launch file, the run script and the startup log:
the arm **moves on its own** after spawn (the `ALIGN` state), and the `INITIAL` state latches
whatever wrench it measures as the bias, so **nobody may touch the robot until `weight lift active`
is logged** — a hand on the tool during settling is baked into every subsequent command.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`franka_weight_lift` is a ROS 1 Noetic (Ubuntu 20.04) catkin package with a single `franka_hw`
effort controller for a Franka Research 3 (FR3). It renders a constant downward force at the end
effector while leaving the end effector free to be moved by hand inside the vertical X-Z plane of
the robot base. See `README.md` for the control law and the reasoning behind it.

## Build / run

This repo is **not** built in place. `compile_package.sh` wipes `~/workspace/catkin_ws_fwl/`, copies
`franka_weight_lift/` into it and `catkin build`s there against `catkin_ws_franka` and
`~/workspace/libfranka/build`. Edits here do not reach the running binary until you re-run the
compile script.

```bash
./scripts/compile_package.sh                     # build
./scripts/run_experiment.sh [force_N] [robot_ip] # real robot, default 10.0 N / 192.168.1.11
```

Code style: Google C++, `IndentWidth: 2`, `ColumnLimit: 100` (`.clang-format`). C++17.

## Architecture

One `controller_interface::MultiInterfaceController` plugin, registered through
`franka_controllers_plugin.xml` and spawned by `controller_manager` — **not** a standalone node, so
there is no `main()` and no ROS topics of its own. Everything is in
`src/weight_lift_controller.cpp` + `include/franka_weight_lift/weight_lift_controller.h`, wired up
by `launch/weight_lift.launch` (which also includes `franka_control.launch`).

Control flow in `update()`: read `franka::RobotState` → build a 6x1 desired wrench in the **robot
base frame** → `tau = J^T * wrench + nullspace + coriolis` → `saturateTorqueRate` → `setCommand`.
A two-state FSM (`INITIAL` → `LIFT`) holds the startup pose stiffly while the external wrench
estimate settles, then switches to the weight-lift law.

Parameters: `config/weight_lift_controller.yaml` is the single source of truth, read in
`readParameters()`. `desired_force`, `k_p` and `k_i` are additionally exposed through
`dynamic_reconfigure` (`cfg/weight_lift_param.cfg`); `init()` seeds the reconfigure server's
namespace from the yaml values so the first reconfigure callback cannot clobber them with the `.cfg`
defaults.

## Conventions

- **Base frame is the task frame.** Free axes are base X (index 0) and base Z (index 2); base Y
  (index 1) and all three rotations are held. `getZeroJacobian` is expressed in the base frame, so
  every wrench assembled in `liftWrench` / `poseHoldWrench` must be too.
- **Wrench signs follow libfranka.** `O_F_ext_hat_K` is the wrench the robot applies *to* the
  environment, in base coordinates, so a downward pull is a negative z entry both when commanding
  and when measuring. Do not "fix" the sign of the `initial_wrench_O_` term: it matches the
  convention documented in `franka/robot_state.h` and used by `seeing_is_enough`.
- **Orientation target is frozen at `starting()`**, never driven to identity — the FR3 flange is
  about `diag(1,-1,-1)` relative to the base when pointing down, and commanding identity would
  rotate the arm as soon as the controller is spawned.
- Anything touching the real robot at 1 kHz: keep it allocation-light, keep the torque saturation,
  and default new force-feedback gains to zero.

## Safety

The free plane has no vertical stiffness, so if the human lets go the arm keeps pulling down.
Virtual walls (relative to the startup pose), the free-plane speed cap and per-joint torque
saturation are the only things bounding that — do not remove them, and keep `wall_*_min <= 0 <=
wall_*_max` so the controller never starts outside its own walls.

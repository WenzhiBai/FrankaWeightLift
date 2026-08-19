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
| base **X**, base **Z** | **free** — no position servo and no damping at all. A speed cap, and virtual walls once a force is being asked for. This is the plane the human works in. |
| base **Y** | held at **y = 0** by a stiff spring/damper, so the tool works in the base's own X-Z plane. |
| all 3 rotations | held at **diag(1, -1, -1)** by a stiff spring/damper: end-effector X along base X, end-effector Y along -base Y, end-effector Z along -base Z, i.e. the tool pointing straight down. |

The last two are *targets*, not startup values: an `ALIGN` state drives the arm onto them right
after the controller is spawned, so **the arm moves on its own at startup**.

On top of that the controller commands `desired_force` newtons along **-Z of the base**, i.e.
towards the earth along gravity.

The commanded wrench (base frame) is

```
w  =  [0, 0, -F]                          commanded pull towards the earth
   +  [0, 0,  k_p*e + k_i*∫e]             optional PI on the transmitted force (gains 0 by default)
   +  initial_wrench                      cancels the wrench already on the flange at startup
   +  [0, -clamp(K_y*y) - D_y*vy, 0]      base Y held at 0: the tool works in the base X-Z plane
   +  [0,0,0, -clamp(K_r*e_r) - D_r*ω]    attitude held at diag(1, -1, -1)
   +  wall(x) , wall(z)                   virtual walls, only while a force is asked for
   +  brake(vx, vz)                       braking above max_free_speed

tau = J^T w + nullspace(q_d) + coriolis   , then rate- and magnitude-saturated
```

### Design decisions worth knowing

- **Nothing acts in the free plane except the commanded force.** Base X and Z carry no stiffness
  and no damping, so what the human feels is `J^T w` and joint friction, nothing else. The two
  remaining terms in the plane are zero until they are needed: the walls only exist while a force
  is being asked for, and the speed cap is zero below `max_free_speed`.
- **The end effector is aligned with the base at startup, not frozen where it was.** The `ALIGN`
  state pulls base Y to 0 and the attitude to `diag(1, -1, -1)`, so the working plane and the tool
  direction are the same on every run instead of depending on where the arm was parked. Base X and Z
  are held where they are while that happens — there is no target for them, the human sets them.
- **Alignment is a snap to target, and the arm moves by itself.** Both targets are commanded
  straight away and the existing stiff spring/damper does the moving. To keep that bounded, the
  *stiffness* part of the lateral hold and of the attitude hold is clamped (`max_lateral_force`,
  `max_rotational_torque`) while the damping term stays outside the clamp — so the move behaves like
  a constant-force pull that still decelerates on arrival, with a terminal speed of
  `max_lateral_force / lateral_damping`. Alignment is refused outright (state `FAULT`, arm holds
  still) if the startup pose is further off than `max_align_y_offset` / `max_align_angle`.
- **The pull is along base -Z, not along the end effector's own Z axis.** They coincide once the
  tool is aligned, and using the base axis guarantees the force is exactly vertical even if the
  attitude is slightly off.
- **Sign convention.** libfranka's `O_F_ext_hat_K` is the wrench the robot applies *to* its
  environment, expressed in base coordinates (see `franka/robot_state.h`), so pulling down is a
  negative z entry both when commanding and when measuring.
- **The startup wrench is measured and cancelled.** An unmodelled tool weight would otherwise add
  to the commanded pull, since the free plane has no vertical stiffness to hold it. The controller
  spends its `INITIAL` state holding the startup pose stiffly in all six DOF until the estimate
  settles, then latches it. **Do not touch the robot until `weight lift active` is logged.**
  Configuring `setLoad` properly is still the better fix; set `compensate_initial_wrench: false` if
  you would rather not have the bias term at all.
- **`desired_force` starts at 0, and the walls follow it.** With no force asked for, the tool can be
  carried anywhere in the plane — the walls are off entirely. The moment a non-zero force is
  requested the walls arm themselves and anchor on the pose the tool is in right then, so the
  working box is always centred on where the work actually happens. Set the force back to 0 and the
  walls are released again (once the rendered force has decayed), so the tool can be repositioned.
- **Force feedback defaults to off.** `k_p = k_i = 0` means pure feedforward `J^T w`, which cannot
  go unstable. Raise `k_p` in steps of ~0.1 with `rqt_reconfigure` to compensate joint friction. The
  integrator is frozen above `integral_freeze_speed` so it cannot wind up while the tool is falling
  with nothing holding it.
- **The nullspace term costs nothing at the tool.** It is projected into the nullspace of `J^T`, so
  it holds the elbow near `q_d_nullspace` without adding end-effector force. That reference is
  re-latched when the align move finishes, so it holds the configuration the aligned pose ended up
  in rather than the one the arm was parked in.
- **Safety.** The free plane has no vertical stiffness, so if the human lets go while a force is
  being rendered the arm keeps pulling down. Three things bound that: virtual walls around the pose
  where the force was applied (`wall_z_min` etc.), a speed cap (`max_free_speed`) and per-joint
  torque saturation. The lower z wall has to hold the commanded pull, so it sags by
  `desired_force / wall_stiffness` (5 mm at 10 N / 2000 N/m).
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

# Run: 0 N to start with, default robot ip, rqt_reconfigure opens with it
./scripts/run_experiment.sh
# Start straight at 25 N downwards, explicit robot ip
./scripts/run_experiment.sh 25.0 192.168.1.11
```

Or directly:

```bash
roslaunch franka_weight_lift weight_lift.launch desired_force:=25.0
```

`desired_force` defaults to **0 N**, and `run_experiment.sh` always opens `rqt_reconfigure`
(`rqt_reconfigure` → `weight_lift_controller`), which is how the controller is meant to be driven:
let it align, move the tool to where the work is, then dial the force up. `k_p` and `k_i` are live
there too. Everything else lives in `franka_weight_lift/config/weight_lift_controller.yaml`, which
is the single source of truth — `init()` seeds the reconfigure server from it, so the yaml wins at
startup.

Before running: release the FR3 brakes and put the robot into `FCI`/execution mode in Desk, as for
any `franka_ros` torque controller. **The arm moves on its own once the controller is spawned** —
it settles the wrench estimate, then aligns itself with the base. Stay clear of it and do not touch
it until `weight lift active` is logged.

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
| tool drifts sideways out of the plane | raise `lateral_stiffness`, or `max_lateral_force` if it is saturating |
| tool twists in the hand | raise `rotational_stiffness` / `max_rotational_torque` |
| free plane drifts or feels lively when released | it is undamped by design; the speed cap is the only brake. Lower `max_free_speed` |
| align move at startup is too brisk | lower `max_lateral_force` / `max_rotational_torque` |
| `refusing to align` at startup | park the arm closer to the base X-Z plane with the tool pointing down, or widen `max_align_y_offset` / `max_align_angle` |
| `alignment did not converge` | raise `max_align_sec`, or loosen `align_position_tolerance` / `align_orientation_tolerance` |
| elbow wanders while dragging | raise `nullspace_stiffness` |
| felt force is less than commanded | raise `k_p` (friction compensation) |
| working volume too small | widen `wall_*`; keep `*_min <= 0 <= *_max` |

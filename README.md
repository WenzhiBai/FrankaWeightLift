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
| base **X**, base **Z** translation | **free** — no position servo and no damping at all. A speed cap, and virtual walls once a force is being asked for. This is the plane the human works in. |
| base **Y** translation | held at **y = 0** by a stiff spring/damper, so the tool works in the base's own X-Z plane. |
| sideways **tip** of the tool axis | held at **0** by a stiff spring/damper: the tool axis (end-effector Z) is kept *inside* the base X-Z plane. This is the only held rotational DOF. |
| tool **tilt** and **spin** | **free** — tilting the tool within the plane and spinning it while it hangs vertically cost exactly nothing. |

Only two DOF are held, and they are *targets*, not startup values: an `ALIGN` state drives the arm
onto them right after the controller is spawned, so **the arm moves on its own at startup**. What is
free is genuinely free — no stiffness, no damping, nothing.

On top of that the controller commands `desired_force` newtons along **-Z of the base**, i.e.
towards the earth along gravity.

The commanded wrench (base frame) is

```
a  =  R * e_z                             the tool axis, in base coordinates
e  =  asin(a_y)                           how far it has tipped out of the base X-Z plane
n  =  normalize(a × y_base)               the direction a torque has to act along to change e

w  =  [0, 0, -F]                          commanded pull towards the earth
   +  [0, 0,  k_p*e_F + k_i*∫e_F]         optional PI on the transmitted force (gains 0 by default)
   +  initial_wrench                      cancels the force *and torque* already on the flange
   +  [0, -clamp(K_y*y) - D_y*vy, 0]      base Y held at 0: the tool works in the base X-Z plane
   +  [0,0,0, (-clamp(K_r*e) - D_r*(ω·n)) * n]   the tool axis held inside that plane
   +  wall(x) , wall(z)                   virtual walls, only while a force is asked for
   +  brake(vx, vz)                       braking above max_free_speed

tau = J^T w + coriolis                    , then rate- and magnitude-saturated
```

`n` never has a base-Y component and vanishes when `e = 0`, which is what makes tilt and spin free:
the torque acts only along the one direction that changes the tip.

### Design decisions worth knowing

- **Nothing acts in the free DOF except the commanded force.** Base X and Z translation carry no
  stiffness and no damping, and neither do the tool's tilt and spin, so what the human feels is
  `J^T w` and joint friction, nothing else. The two remaining terms in the plane are zero until they
  are needed: the walls only exist while a force is being asked for, and the speed cap is zero below
  `max_free_speed`.
- **Only one rotational DOF is held: the sideways tip.** The constraint is geometric — the tool axis
  stays *in* the base X-Z plane — rather than an attitude target. Tilting the tool within the plane
  is free at any angle, and so is spinning it while it hangs vertically; both produce exactly zero
  torque. The price is that yawing an *already tilted* tool is resisted, because a tilted-and-yawed
  tool axis necessarily leaves the plane: a 30° tilt plus a 30° yaw tips the axis 14.5° out of plane
  and meets the full `max_rotational_torque`.
- **The tip is measured exactly, not read off an attitude error.** Taking the base-X entry of a
  full attitude error vector instead would look simpler, but the axis-angle error mixes its
  components: a 30° tilt combined with a 30° yaw would report an 8° sideways tip that is not there.
- **Alignment removes the y offset and the tip, nothing else.** The `ALIGN` state leaves whatever
  tilt and spin the arm was parked with alone, the same way base X and Z translation are held where
  they are rather than driven to a target.
- **Alignment is a snap to target, and the arm moves by itself.** Both targets are commanded
  straight away and the existing stiff spring/damper does the moving. To keep that bounded, the
  *stiffness* part of the lateral hold and of the tip hold is clamped (`max_lateral_force`,
  `max_rotational_torque`) while the damping term stays outside the clamp — so the move behaves like
  a constant-force pull that still decelerates on arrival, with a terminal speed of
  `max_lateral_force / lateral_damping`. Alignment is refused outright (state `FAULT`, arm holds
  still) if the startup pose is further off than `max_align_y_offset` / `max_align_angle`.
- **The pull is along base -Z, not along the end effector's own Z axis.** They coincide while the
  tool hangs vertically, and using the base axis guarantees the force stays exactly vertical however
  the human tilts the tool.
- **Sign convention.** libfranka's `O_F_ext_hat_K` is the wrench the robot applies *to* its
  environment, expressed in base coordinates (see `franka/robot_state.h`), so pulling down is a
  negative z entry both when commanding and when measuring.
- **The startup wrench is measured and cancelled, torque included.** An unmodelled tool weight
  would otherwise add to the commanded pull, since the free plane has no vertical stiffness to hold
  it — and its *torque* would turn the tool over, since the tilt and spin have no stiffness either.
  The controller spends its `INITIAL` state holding the startup pose stiffly in all six DOF, and
  latches the *mean* of the estimate over a window in which it held still (`min_settle_sec` of
  samples within `settle_wrench_tolerance` of the running mean; any excursion restarts the window).
  **Do not touch the robot until `weight lift active` is logged** — a hand resting steadily on the
  tool is indistinguishable from a heavier tool, and nothing in the estimate can separate them.
  Set `compensate_initial_wrench: false` if you would rather not have the bias term at all.
- **The bias is a constant, so it only holds where it was measured.** Tool weight is a constant
  vector in base coordinates and is compensated well. Joint friction and arm model error are *not*:
  they are configuration-dependent, and they are the dominant error in the force the human actually
  feels. The tool's torque is attitude-dependent too, now that tilt and spin are free. In order of
  effect, the ways to get an accurate force are: get `setLoad` right (it fixes the estimator's
  baseline at every configuration, not just one); tick `recalibrate` in `rqt_reconfigure` at the
  pose you are about to work at; then raise `k_p` so the loop keeps correcting what is left. Without
  a wrist force/torque sensor, the joint-torque estimate is the accuracy ceiling.
- **`desired_force` starts at 0, and the walls follow it.** With no force asked for, the tool can be
  carried anywhere in the plane — the walls are off entirely. The moment a non-zero force is
  requested the walls arm themselves and anchor on the pose the tool is in right then, so the
  working box is always centred on where the work actually happens. Set the force back to 0 and the
  walls are released again (once the rendered force has decayed), so the tool can be repositioned.
- **Force feedback defaults to off.** `k_p = k_i = 0` means pure feedforward `J^T w`, which cannot
  go unstable. Raise `k_p` in steps of ~0.1 with `rqt_reconfigure` to compensate joint friction. The
  integrator is frozen above `integral_freeze_speed` so it cannot wind up while the tool is falling
  with nothing holding it.
- **There is no nullspace term.** In theory a nullspace torque is invisible at the end effector;
  in practice it was felt on the tool, so it is gone. The redundant DOF is therefore uncontrolled:
  nothing holds or damps the elbow while the tool is dragged around.
- **Safety.** The free plane has no vertical stiffness, so if the human lets go while a force is
  being rendered the arm keeps pulling down. Three things bound that: virtual walls around the pose
  where the force was applied (`wall_z_min` etc.), a speed cap (`max_free_speed`) and per-joint
  torque saturation. The lower z wall has to hold the commanded pull, so it sags by
  `desired_force / wall_stiffness` (5 mm at 10 N / 2000 N/m). The two free rotations have no cap of
  any kind, and neither does the redundant DOF.
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
let it align, move the tool to where the work is, tick **`recalibrate`** and let go of the tool
while it re-zeroes, then dial the force up. `k_p` and `k_i` are live there too.

`recalibrate` is a momentary trigger — it acts on the false → true edge, so untick it to arm it
again. It is only accepted with `desired_force` at 0, the ramp decayed and the arm at rest; while it
runs the controller holds the current pose stiffly in all six DOF and **nobody may touch the
robot**, exactly as at startup. Everything else lives in `franka_weight_lift/config/weight_lift_controller.yaml`, which
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
| tool tips sideways out of the plane | raise `rotational_stiffness` / `max_rotational_torque` |
| tool tips or spins under its own weight | `setLoad` is wrong; the torque bias only cancels it at the attitude where it was measured. Tick `recalibrate` at the working attitude |
| felt force is off by a newton or two | tick `recalibrate` at the working pose, then raise `k_p`. A constant bias cannot track configuration-dependent friction |
| `the wrench estimate never held still` | the estimate is noisier than `settle_wrench_tolerance`; raise it, or accept the averaged fallback |
| yawing the tool feels sticky | expected once the tool is tilted — that motion takes the tool axis out of the plane. Lower `rotational_stiffness` / `rotational_damping` to soften it |
| free plane drifts or feels lively when released | it is undamped by design; the speed cap is the only brake. Lower `max_free_speed` |
| align move at startup is too brisk | lower `max_lateral_force` / `max_rotational_torque` |
| `refusing to align` at startup | park the arm closer to the base X-Z plane with the tool hanging into it, or widen `max_align_y_offset` / `max_align_angle` |
| `alignment did not converge` | raise `max_align_sec`, or loosen `align_position_tolerance` / `align_orientation_tolerance` |
| `the tool axis points along base Y` | the tool is 90° out of plane, where the hold is singular. Reposition and respawn |
| felt force is less than commanded | raise `k_p` (friction compensation) |
| working volume too small | widen `wall_*`; keep `*_min <= 0 <= *_max` |

#include "franka_weight_lift/weight_lift_controller.h"

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace franka_weight_lift {

namespace {
// The align move counts as finished only once the arm has actually stopped, so
// a fast pass through the target cannot release the free plane mid-swing.
constexpr double kAlignSettleSpeed = 0.01;  // m/s
constexpr double kAlignSettleRate = 0.05;   // rad/s
// Below this the commanded force has effectively decayed to zero, which is when
// the virtual walls may be released again.
constexpr double kForceOffThreshold = 0.05;  // N
// |tool axis x y_base| below this means the tool axis lies along base Y, where
// no torque can bring it back into the plane.
constexpr double kToolAxisSingularity = 1e-3;
}  // namespace

bool WeightLiftController::init(hardware_interface::RobotHW* robot_hw,
                                ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("WeightLiftController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR_STREAM(
        "WeightLiftController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!readParameters(node_handle)) {
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("WeightLiftController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "WeightLiftController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("WeightLiftController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "WeightLiftController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("WeightLiftController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("WeightLiftController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Seed the reconfigure server's namespace with the yaml values so the config
  // file stays the single source of truth: dynamic_reconfigure initialises its
  // config from the parameters it finds there, and its first callback would
  // otherwise overwrite what we just read with the .cfg defaults.
  dynamic_reconfigure_weight_lift_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_weight_lift_param_node");
  dynamic_reconfigure_weight_lift_param_node_.setParam("desired_force", target_force_);
  dynamic_reconfigure_weight_lift_param_node_.setParam("k_p", k_p_);
  dynamic_reconfigure_weight_lift_param_node_.setParam("k_i", k_i_);
  dynamic_server_weight_lift_param_ =
      std::make_unique<dynamic_reconfigure::Server<franka_weight_lift::weight_lift_paramConfig>>(
          dynamic_reconfigure_weight_lift_param_node_);
  dynamic_server_weight_lift_param_->setCallback(
      boost::bind(&WeightLiftController::weightLiftParamCallback, this, _1, _2));

  return true;
}

bool WeightLiftController::readParameters(ros::NodeHandle& node_handle) {
  // Commanded force. target_force_ is a magnitude in newtons; it is rendered
  // along -Z of the robot base, i.e. towards the earth. It defaults to 0, so
  // the tool starts out free in the plane.
  if (!node_handle.getParam("desired_force", target_force_)) {
    ROS_ERROR_STREAM("WeightLiftController: Could not read parameter desired_force");
    return false;
  }
  if (target_force_ < 0.0) {
    ROS_ERROR_STREAM("WeightLiftController: desired_force must be a non-negative magnitude, got "
                     << target_force_);
    return false;
  }
  node_handle.param("force_filter_gain", force_filter_gain_, force_filter_gain_);
  node_handle.param("k_p", k_p_, k_p_);
  node_handle.param("k_i", k_i_, k_i_);
  node_handle.param("max_force_correction", max_force_correction_, max_force_correction_);
  node_handle.param("max_force_integral", max_force_integral_, max_force_integral_);
  node_handle.param("integral_freeze_speed", integral_freeze_speed_, integral_freeze_speed_);

  node_handle.param("lateral_stiffness", lateral_stiffness_, lateral_stiffness_);
  node_handle.param("lateral_damping", lateral_damping_, lateral_damping_);
  node_handle.param("max_lateral_force", max_lateral_force_, max_lateral_force_);
  node_handle.param("rotational_stiffness", rotational_stiffness_, rotational_stiffness_);
  node_handle.param("rotational_damping", rotational_damping_, rotational_damping_);
  node_handle.param("max_rotational_torque", max_rotational_torque_, max_rotational_torque_);

  node_handle.param("wall_x_min", wall_x_min_, wall_x_min_);
  node_handle.param("wall_x_max", wall_x_max_, wall_x_max_);
  node_handle.param("wall_z_min", wall_z_min_, wall_z_min_);
  node_handle.param("wall_z_max", wall_z_max_, wall_z_max_);
  node_handle.param("wall_stiffness", wall_stiffness_, wall_stiffness_);
  node_handle.param("wall_damping", wall_damping_, wall_damping_);

  node_handle.param("max_free_speed", max_free_speed_, max_free_speed_);
  node_handle.param("brake_damping", brake_damping_, brake_damping_);

  node_handle.param("compensate_initial_wrench", compensate_initial_wrench_,
                    compensate_initial_wrench_);
  node_handle.param("settle_wrench_tolerance", settle_wrench_tolerance_, settle_wrench_tolerance_);
  node_handle.param("min_settle_sec", min_settle_sec_, min_settle_sec_);
  node_handle.param("max_settle_sec", max_settle_sec_, max_settle_sec_);

  node_handle.param("align_position_tolerance", align_position_tolerance_,
                    align_position_tolerance_);
  node_handle.param("align_orientation_tolerance", align_orientation_tolerance_,
                    align_orientation_tolerance_);
  node_handle.param("max_align_sec", max_align_sec_, max_align_sec_);
  node_handle.param("max_align_y_offset", max_align_y_offset_, max_align_y_offset_);
  node_handle.param("max_align_angle", max_align_angle_, max_align_angle_);

  if (wall_x_min_ > 0.0 || wall_x_max_ < 0.0 || wall_z_min_ > 0.0 || wall_z_max_ < 0.0) {
    ROS_ERROR_STREAM(
        "WeightLiftController: the virtual walls are relative to the pose where the force is "
        "first requested, so the *_min bounds must be <= 0 and the *_max bounds >= 0, otherwise "
        "the controller arms them outside its own walls");
    return false;
  }

  return true;
}

void WeightLiftController::starting(const ros::Time& time) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  const Eigen::Isometry3d initial_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Hold the pose we start in, in all six DOF, while the wrench estimate
  // settles. beginAlignment() then moves the base Y target onto 0 and hands the
  // rotations over to the tool-axis constraint.
  hold_position_ = initial_pose.translation();
  orientation_d_ = Eigen::Quaterniond(initial_pose.rotation());

  // Ramp up from zero force; the filter in update() does the ramping.
  desired_force_ = 0.0;
  force_integral_ = 0.0;

  // Nothing measured yet, and drop any recalibration request left from a
  // previous spawn.
  bias_samples_ = 0;
  recalibrate_requested_ = false;

  // The walls only exist while a force is being rendered; they anchor themselves
  // the moment a non-zero force is requested.
  walls_armed_ = false;
  wall_origin_ = initial_pose.translation();

  initial_wrench_O_ = Eigen::Matrix<double, 6, 1>::Map(robot_state.O_F_ext_hat_K.data());
  if (!compensate_initial_wrench_) {
    initial_wrench_O_.setZero();
  }

  transitionToState(ControllerState::INITIAL, time);
  ROS_WARN_STREAM(
      "WeightLiftController: settling the external wrench estimate, then aligning the end "
      "effector with the robot base - THE ARM MOVES ON ITS OWN. Stay clear and do NOT touch the "
      "robot until 'weight lift active' is logged");
}

void WeightLiftController::update(const ros::Time& time, const ros::Duration& period) {
  // Get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // Convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  // Wrench the robot currently applies to its environment, in base coordinates.
  Eigen::Map<Eigen::Matrix<double, 6, 1>> measured_wrench_O(robot_state.O_F_ext_hat_K.data());

  const Eigen::Isometry3d pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  const Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

  Eigen::Matrix<double, 6, 1> wrench_d = Eigen::Matrix<double, 6, 1>::Zero();
  ControllerState next_state = current_state_;
  switch (current_state_) {
    case ControllerState::INITIAL:
      // Stay put in all six DOF while the wrench estimate settles.
      wrench_d = poseHoldWrench(pose, velocity);
      next_state = handleInitialState(measured_wrench_O, time);
      if (next_state == ControllerState::ALIGN) {
        next_state = beginAlignment(pose);
      }
      break;

    case ControllerState::ALIGN:
      // Base Y is pulled to 0 and the tool axis into the base X-Z plane, while
      // base X and Z stay put and the tool's tilt and spin are left alone.
      wrench_d = alignWrench(pose, velocity);
      next_state = handleAlignState(pose, velocity, time);
      break;

    case ControllerState::LIFT:
      wrench_d = liftWrench(pose, velocity, measured_wrench_O, period);
      // After the wrench, so the hold targets beginCalibration() overwrites
      // cannot affect the command this tick.
      if (recalibrate_requested_) {
        recalibrate_requested_ = false;
        next_state = beginCalibration(pose, velocity);
      }
      break;

    case ControllerState::CALIBRATE:
      // Hold the pose stiffly in all six DOF, so nothing drifts into the
      // measurement, and re-measure the bias where the work actually happens.
      wrench_d = poseHoldWrench(pose, velocity);
      if (updateWrenchBias(measured_wrench_O, time)) {
        // Back onto the aligned base Y target, and drop the integral that was
        // wound up against the old bias.
        hold_position_(kLockedAxisY) = 0.0;
        force_integral_ = 0.0;
        next_state = ControllerState::LIFT;
      }
      break;

    case ControllerState::FAULT:
      ROS_ERROR_STREAM_THROTTLE(5.0,
                                "WeightLiftController: alignment refused, holding the startup "
                                "pose. Move the arm closer to the base X-Z plane with the tool "
                                "hanging into that plane, then respawn the controller");
      wrench_d = poseHoldWrench(pose, velocity);
      break;

    case ControllerState::UNKNOWN:
    default:
      ROS_ERROR_STREAM("WeightLiftController: Unknown state, holding still");
      wrench_d = poseHoldWrench(pose, velocity);
  }

  Eigen::Matrix<double, 7, 1> tau_d = jacobian.transpose() * wrench_d + coriolis;
  tau_d = saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  ROS_INFO_STREAM_THROTTLE(
      0.5,
      "state: " << stateToString(current_state_) << ", commanded force: " << desired_force_
                << "N, measured z force: "
                << measured_wrench_O(kFreeAxisZ) - initial_wrench_O_(kFreeAxisZ) << "N, position x "
                << pose.translation()(kFreeAxisX) << "m, y " << pose.translation()(kLockedAxisY)
                << "m, z " << pose.translation()(kFreeAxisZ) << "m, tool axis tip "
                << toolAxisTip(pose).angle << "rad, walls " << (walls_armed_ ? "armed" : "off"));

  // Ramp the commanded force towards the target, so both the startup ramp and
  // live dynamic_reconfigure changes are smooth.
  desired_force_ = force_filter_gain_ * target_force_ + (1.0 - force_filter_gain_) * desired_force_;

  if (next_state != current_state_) {
    transitionToState(next_state, time);
  }
}

WeightLiftController::ToolAxisTip WeightLiftController::toolAxisTip(
    const Eigen::Isometry3d& pose) const {
  // The tool axis is end-effector Z in base coordinates. Holding it inside the
  // base X-Z plane means driving its base-Y component to zero.
  const Eigen::Vector3d tool_axis = pose.rotation().col(2);

  ToolAxisTip tip;
  tip.angle = std::asin(std::clamp(tool_axis(kLockedAxisY), -1.0, 1.0));
  // d(tool_axis_y)/dt = omega . (tool_axis x y_base), so that cross product is
  // the direction a torque has to act along to change the tip. It never has a
  // base-Y entry, which is why tilting the tool inside the plane is always free,
  // and it vanishes exactly when the tool axis lies along base Y.
  Eigen::Vector3d axis(-tool_axis(kFreeAxisZ), 0.0, tool_axis(kFreeAxisX));
  const double norm = axis.norm();
  if (norm > kToolAxisSingularity) {
    tip.axis = axis / norm;
  }
  return tip;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::toolAxisHoldWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();

  const ToolAxisTip tip = toolAxisTip(pose);
  if (tip.axis.isZero()) {
    ROS_ERROR_STREAM_THROTTLE(1.0,
                              "WeightLiftController: the tool axis points along base Y, where no "
                              "torque can bring it back into the plane - reposition the arm");
    return wrench;
  }

  // Stiffness clamped, damping outside the clamp, as for the lateral hold. Only
  // the angular velocity along the held direction is damped, so the tool's tilt
  // and spin cost nothing.
  const double spring = std::clamp(-rotational_stiffness_ * tip.angle, -max_rotational_torque_,
                                   max_rotational_torque_);
  const double damping = -rotational_damping_ * velocity.tail(3).dot(tip.axis);
  wrench.tail(3) = (spring + damping) * tip.axis;
  return wrench;
}

Eigen::Vector3d WeightLiftController::translationHoldForce(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Vector3d force;
  for (int axis = 0; axis < 3; ++axis) {
    force(axis) = lateralHoldForce(pose.translation()(axis) - hold_position_(axis), velocity(axis));
  }
  return force;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::poseHoldWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Matrix<double, 6, 1> wrench = orientationHoldWrench(pose, velocity);
  wrench.head(3) = translationHoldForce(pose, velocity);
  return wrench;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::alignWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Matrix<double, 6, 1> wrench = toolAxisHoldWrench(pose, velocity);
  // The tool's own weight torque, so its now-free tilt and spin do not fall away
  // while the align move runs.
  wrench.tail(3) += initial_wrench_O_.tail(3);
  wrench.head(3) += translationHoldForce(pose, velocity);
  return wrench;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::liftWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity,
    const Eigen::Matrix<double, 6, 1>& measured_wrench_O, const ros::Duration& period) {
  Eigen::Matrix<double, 6, 1> wrench = toolAxisHoldWrench(pose, velocity);

  // Cancel whatever was already pulling and twisting the flange at startup (an
  // unmodelled tool weight, mostly), so the free DOF are neutral apart from the
  // force we command. The torque part matters as much as the force part now that
  // the tool's tilt and spin are free: without it an off-axis tool CoM would
  // simply turn the tool over.
  wrench += initial_wrench_O_;

  // 1. The commanded pull towards the earth, along -Z of the robot base.
  wrench(kFreeAxisZ) += -desired_force_;

  // 2. Optional PI on the force actually transmitted to the environment. Both
  //    the command and the measurement are "force applied by the robot on the
  //    environment", so both are negative when pulling down.
  const double measured_force = measured_wrench_O(kFreeAxisZ) - initial_wrench_O_(kFreeAxisZ);
  const double force_error = -desired_force_ - measured_force;
  // Only integrate while quasi-static, i.e. while something is actually holding
  // the tool. If the human lets go, the free axis accelerates, no reaction force
  // is measured and an unfrozen integrator would wind up against thin air.
  if (std::abs(velocity(kFreeAxisZ)) < integral_freeze_speed_) {
    force_integral_ = std::clamp(force_integral_ + period.toSec() * force_error,
                                 -max_force_integral_, max_force_integral_);
  }
  const double correction = std::clamp(k_p_ * force_error + k_i_ * force_integral_,
                                       -max_force_correction_, max_force_correction_);
  wrench(kFreeAxisZ) += correction;

  // 3. Hold base Y at 0, so the end effector works in the base's own X-Z plane.
  wrench(kLockedAxisY) += lateralHoldForce(
      pose.translation()(kLockedAxisY) - hold_position_(kLockedAxisY), velocity(kLockedAxisY));

  // 4. Virtual walls, so a release under load cannot run the arm into its joint
  //    limits. They only exist while a force is being asked for: below that the
  //    plane is unbounded, so the tool can be carried to wherever the work is,
  //    and the walls anchor themselves at the pose where the force is applied.
  if (!walls_armed_ && target_force_ > 0.0) {
    wall_origin_ = pose.translation();
    walls_armed_ = true;
    ROS_INFO_STREAM("WeightLiftController: virtual walls armed at x "
                    << wall_origin_(kFreeAxisX) << "m, z " << wall_origin_(kFreeAxisZ) << "m");
  } else if (walls_armed_ && target_force_ == 0.0 && desired_force_ < kForceOffThreshold) {
    // Only once the rendered force has decayed too, so the arm is never left
    // unbounded while it is still pulling.
    walls_armed_ = false;
    ROS_INFO_STREAM("WeightLiftController: force back to zero, virtual walls released");
  }
  if (walls_armed_) {
    wrench(kFreeAxisX) += wallForce(pose.translation()(kFreeAxisX) - wall_origin_(kFreeAxisX),
                                    velocity(kFreeAxisX), wall_x_min_, wall_x_max_);
    wrench(kFreeAxisZ) += wallForce(pose.translation()(kFreeAxisZ) - wall_origin_(kFreeAxisZ),
                                    velocity(kFreeAxisZ), wall_z_min_, wall_z_max_);
  }

  // 5. Speed cap in the free plane: brake whatever exceeds max_free_speed_. This
  //    is the only velocity-dependent term left in the free plane, and it is
  //    zero below the cap, so it costs nothing while the tool is hand-guided.
  //    There is no equivalent cap on the two free rotations.
  const Eigen::Vector2d free_velocity(velocity(kFreeAxisX), velocity(kFreeAxisZ));
  const double free_speed = free_velocity.norm();
  if (free_speed > max_free_speed_) {
    const Eigen::Vector2d brake =
        -brake_damping_ * (free_speed - max_free_speed_) * (free_velocity / free_speed);
    wrench(kFreeAxisX) += brake(0);
    wrench(kFreeAxisZ) += brake(1);
  }

  return wrench;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::orientationHoldWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Quaterniond orientation(pose.rotation());
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  const Eigen::AngleAxisd error_angle_axis(orientation.inverse() * orientation_d_);
  // Rotation error expressed in the base frame, to match the zero Jacobian.
  const Eigen::Vector3d error =
      -(pose.rotation() * (error_angle_axis.angle() * error_angle_axis.axis()));

  // The stiffness term is clamped, the damping term stays outside the clamp, as
  // for the lateral hold.
  Eigen::Vector3d torque = -rotational_stiffness_ * error;
  const double torque_norm = torque.norm();
  if (torque_norm > max_rotational_torque_) {
    torque *= max_rotational_torque_ / torque_norm;
  }

  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  wrench.tail(3) = torque - rotational_damping_ * velocity.tail(3);
  return wrench;
}

bool WeightLiftController::updateWrenchBias(const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                                            const ros::Time& time) {
  if (bias_samples_ == 0) {
    bias_sum_ = measured_wrench_O;
    bias_samples_ = 1;
    bias_window_start_ = time;
    return false;
  }

  // Any excursion from the running mean restarts the window, so what gets
  // latched is always a mean over a stretch where nothing was moving. A hand
  // resting steadily on the tool is indistinguishable from a heavier tool,
  // though - that is why nobody may touch the robot while this runs.
  const Eigen::Matrix<double, 6, 1> mean = bias_sum_ / bias_samples_;
  if ((measured_wrench_O - mean).norm() > settle_wrench_tolerance_) {
    bias_sum_ = measured_wrench_O;
    bias_samples_ = 1;
    bias_window_start_ = time;
    return false;
  }
  bias_sum_ += measured_wrench_O;
  ++bias_samples_;

  const double stable = (time - bias_window_start_).toSec();
  if (stable >= min_settle_sec_) {
    initial_wrench_O_ = bias_sum_ / bias_samples_;
    ROS_INFO_STREAM("WeightLiftController: wrench bias latched over "
                    << stable << "s (" << bias_samples_ << " samples): force "
                    << initial_wrench_O_.head(3).transpose() << "N, torque "
                    << initial_wrench_O_.tail(3).transpose() << "Nm");
    return true;
  }
  if ((time - state_entry_time_).toSec() >= max_settle_sec_) {
    initial_wrench_O_ = bias_sum_ / bias_samples_;
    ROS_WARN_STREAM("WeightLiftController: the wrench estimate never held still for "
                    << min_settle_sec_ << "s within " << max_settle_sec_ << "s; using the mean of "
                    << "the last " << stable << "s (" << bias_samples_ << " samples): force "
                    << initial_wrench_O_.head(3).transpose() << "N, torque "
                    << initial_wrench_O_.tail(3).transpose()
                    << "Nm. Raise settle_wrench_tolerance if the estimate is only noisy");
    return true;
  }
  return false;
}

double WeightLiftController::lateralHoldForce(double offset, double velocity) const {
  // The stiffness term is clamped so a large offset - above all the y offset the
  // ALIGN state has to remove - cannot slam the arm across it. The damping term
  // stays outside the clamp, so it still bounds the speed of that move
  // (terminal speed = max_lateral_force / lateral_damping) and still decelerates
  // the arm as it arrives.
  return std::clamp(-lateral_stiffness_ * offset, -max_lateral_force_, max_lateral_force_) -
         lateral_damping_ * velocity;
}

double WeightLiftController::wallForce(double offset, double velocity, double min,
                                       double max) const {
  if (offset > max) {
    return -wall_stiffness_ * (offset - max) - wall_damping_ * velocity;
  }
  if (offset < min) {
    return -wall_stiffness_ * (offset - min) - wall_damping_ * velocity;
  }
  return 0.0;
}

Eigen::Matrix<double, 7, 1> WeightLiftController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    // Rate limit
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    // Absolute limit
    tau_d_saturated[i] = std::max(std::min(tau_d_saturated[i], tau_max_[i]), -tau_max_[i]);
  }
  return tau_d_saturated;
}

void WeightLiftController::weightLiftParamCallback(
    franka_weight_lift::weight_lift_paramConfig& config, uint32_t /*level*/) {
  target_force_ = config.desired_force;
  k_p_ = config.k_p;
  k_i_ = config.k_i;
  ROS_INFO_STREAM("WeightLiftController: desired_force " << target_force_ << "N, k_p " << k_p_
                                                         << ", k_i " << k_i_);
  // Momentary trigger: only the false -> true edge counts, so the checkbox has
  // to be unticked again before it can fire a second time.
  if (config.recalibrate && !last_recalibrate_) {
    recalibrate_requested_ = true;
  }
  last_recalibrate_ = config.recalibrate;
  // The lower z wall has to hold the commanded pull, so its steady-state
  // penetration is desired_force / wall_stiffness. Warn if that is coarse.
  const double wall_penetration = target_force_ / std::max(wall_stiffness_, 1.0);
  if (wall_penetration > 0.05) {
    ROS_WARN_STREAM("WeightLiftController: at " << target_force_ << "N the lower z wall will sag "
                                                << wall_penetration * 1000.0
                                                << "mm; raise wall_stiffness");
  }
}

std::string WeightLiftController::stateToString(ControllerState state) const {
  switch (state) {
    case ControllerState::UNKNOWN:
      return "UNKNOWN";
    case ControllerState::INITIAL:
      return "INITIAL";
    case ControllerState::ALIGN:
      return "ALIGN";
    case ControllerState::LIFT:
      return "LIFT";
    case ControllerState::CALIBRATE:
      return "CALIBRATE";
    case ControllerState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

void WeightLiftController::transitionToState(const ControllerState& new_state,
                                             const ros::Time& time) {
  previous_state_ = current_state_;
  current_state_ = new_state;
  state_entry_time_ = time;

  ROS_INFO_STREAM("WeightLiftController state transition: "
                  << stateToString(previous_state_) << " -> " << stateToString(current_state_));
}

WeightLiftController::ControllerState WeightLiftController::handleInitialState(
    const Eigen::Matrix<double, 6, 1>& measured_wrench_O, const ros::Time& time) {
  if (!compensate_initial_wrench_) {
    ROS_WARN_STREAM(
        "WeightLiftController: initial wrench compensation disabled, aligning immediately");
    return ControllerState::ALIGN;
  }

  if (updateWrenchBias(measured_wrench_O, time)) {
    return ControllerState::ALIGN;
  }
  return ControllerState::INITIAL;
}

WeightLiftController::ControllerState WeightLiftController::beginAlignment(
    const Eigen::Isometry3d& pose) {
  const double y_offset = pose.translation()(kLockedAxisY);
  const double tip_angle = toolAxisTip(pose).angle;

  // Both targets are commanded straight away, so refuse to start from a pose
  // that would turn the align move into a long unsupervised travel.
  if (std::abs(y_offset) > max_align_y_offset_ || std::abs(tip_angle) > max_align_angle_) {
    ROS_ERROR_STREAM("WeightLiftController: refusing to align, the end effector starts "
                     << y_offset << "m off the base X-Z plane with the tool axis tipped "
                     << tip_angle << "rad out of it (limits " << max_align_y_offset_ << "m and "
                     << max_align_angle_ << "rad)");
    return ControllerState::FAULT;
  }

  // Only the base Y target moves. The tool-axis hold that ALIGN and LIFT use has
  // no attitude target to set, and base X and Z stay where the arm is.
  hold_position_(kLockedAxisY) = 0.0;
  ROS_INFO_STREAM("WeightLiftController: aligning with the robot base, removing "
                  << y_offset << "m of y offset and " << tip_angle
                  << "rad of tool-axis tip; the tool's tilt and spin are left as they are");
  return ControllerState::ALIGN;
}

WeightLiftController::ControllerState WeightLiftController::beginCalibration(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) {
  if (!compensate_initial_wrench_) {
    ROS_WARN_STREAM(
        "WeightLiftController: initial wrench compensation is disabled, ignoring the "
        "recalibration request");
    return ControllerState::LIFT;
  }
  // A bias measured while we are still pulling would bake in the reaction to
  // our own pull, or the hand holding the tool against it.
  if (target_force_ > 0.0 || desired_force_ > kForceOffThreshold) {
    ROS_WARN_STREAM(
        "WeightLiftController: recalibration needs desired_force at 0 and the ramp "
        "decayed (still rendering "
        << desired_force_ << "N), ignoring the request");
    return ControllerState::LIFT;
  }
  if (velocity.head(3).norm() > kAlignSettleSpeed || velocity.tail(3).norm() > kAlignSettleRate) {
    ROS_WARN_STREAM(
        "WeightLiftController: recalibration needs the arm at rest, ignoring the request");
    return ControllerState::LIFT;
  }

  // Hold the pose we are in right now, in all six DOF, including the free ones.
  hold_position_ = pose.translation();
  orientation_d_ = Eigen::Quaterniond(pose.rotation());
  bias_samples_ = 0;
  ROS_WARN_STREAM(
      "WeightLiftController: recalibrating the wrench bias at this pose - do NOT touch the robot "
      "until the new bias is logged");
  return ControllerState::CALIBRATE;
}

WeightLiftController::ControllerState WeightLiftController::handleAlignState(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity,
    const ros::Time& time) {
  const double y_error = std::abs(pose.translation()(kLockedAxisY) - hold_position_(kLockedAxisY));

  const ToolAxisTip tip = toolAxisTip(pose);
  // Only the rate of the held DOF, since the tool's tilt and spin are free and
  // may well still be moving.
  const double tip_rate = tip.axis.isZero() ? 0.0 : std::abs(velocity.tail(3).dot(tip.axis));
  const bool stopped = velocity.head(3).norm() < kAlignSettleSpeed && tip_rate < kAlignSettleRate;

  if (y_error < align_position_tolerance_ && std::abs(tip.angle) < align_orientation_tolerance_ &&
      stopped) {
    ROS_INFO_STREAM("WeightLiftController: weight lift active, aligned with the robot base ("
                    << y_error * 1000.0 << "mm and " << tip.angle
                    << "rad residual). The end effector is free in the base X-Z plane, and so are "
                       "the tool's tilt and spin; the virtual walls anchor themselves when a "
                       "non-zero desired_force is asked for");
    return ControllerState::LIFT;
  }
  if ((time - state_entry_time_).toSec() >= max_align_sec_) {
    ROS_WARN_STREAM("WeightLiftController: alignment did not converge within "
                    << max_align_sec_ << "s (" << y_error << "m and " << tip.angle
                    << "rad residual), weight lift active anyway - base Y and the tool-axis tip "
                       "stay servoed");
    return ControllerState::LIFT;
  }

  return ControllerState::ALIGN;
}

}  // namespace franka_weight_lift

PLUGINLIB_EXPORT_CLASS(franka_weight_lift::WeightLiftController,
                       controller_interface::ControllerBase)

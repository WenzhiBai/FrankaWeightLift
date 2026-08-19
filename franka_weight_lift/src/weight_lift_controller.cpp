#include "franka_weight_lift/weight_lift_controller.h"

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "franka_weight_lift/pseudo_inversion.h"

namespace franka_weight_lift {

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
  // along -Z of the robot base, i.e. towards the earth.
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
  node_handle.param("rotational_stiffness", rotational_stiffness_, rotational_stiffness_);
  node_handle.param("rotational_damping", rotational_damping_, rotational_damping_);
  node_handle.param("free_damping", free_damping_, free_damping_);
  node_handle.param("nullspace_stiffness", nullspace_stiffness_, nullspace_stiffness_);

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

  if (wall_x_min_ > 0.0 || wall_x_max_ < 0.0 || wall_z_min_ > 0.0 || wall_z_max_ < 0.0) {
    ROS_ERROR_STREAM(
        "WeightLiftController: the virtual walls are relative to the startup pose, so the "
        "*_min bounds must be <= 0 and the *_max bounds >= 0, otherwise the controller starts "
        "outside its own walls");
    return false;
  }
  // The lower z wall has to hold the commanded pull, so its steady-state
  // penetration is desired_force / wall_stiffness. Warn if that is coarse.
  const double wall_penetration = target_force_ / std::max(wall_stiffness_, 1.0);
  if (wall_penetration > 0.05) {
    ROS_WARN_STREAM("WeightLiftController: at " << target_force_ << "N the lower z wall will sag "
                                                << wall_penetration * 1000.0
                                                << "mm; raise wall_stiffness");
  }

  return true;
}

void WeightLiftController::starting(const ros::Time& time) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  const Eigen::Isometry3d initial_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  initial_position_ = initial_pose.translation();
  // Orientation target is frozen at whatever attitude the end effector has now,
  // so spawning the controller never produces a reorientation motion. From here
  // on the end-effector frame keeps this fixed attitude relative to the base.
  orientation_d_ = Eigen::Quaterniond(initial_pose.rotation());

  q_d_nullspace_ = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());

  // Ramp up from zero force; the filter in update() does the ramping.
  desired_force_ = 0.0;
  force_integral_ = 0.0;

  initial_wrench_O_ = Eigen::Matrix<double, 6, 1>::Map(robot_state.O_F_ext_hat_K.data());
  if (!compensate_initial_wrench_) {
    initial_wrench_O_.setZero();
  }

  transitionToState(ControllerState::INITIAL, time);
  ROS_WARN_STREAM(
      "WeightLiftController: settling the external wrench estimate - do NOT touch the robot until "
      "'weight lift active' is logged");
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
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
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
      break;

    case ControllerState::LIFT:
      wrench_d = liftWrench(pose, velocity, measured_wrench_O, period);
      break;

    case ControllerState::UNKNOWN:
    default:
      ROS_ERROR_STREAM("WeightLiftController: Unknown state, holding still");
      wrench_d = poseHoldWrench(pose, velocity);
  }

  Eigen::Matrix<double, 7, 1> tau_d =
      jacobian.transpose() * wrench_d + nullspaceTorque(jacobian, q, dq) + coriolis;
  tau_d = saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  ROS_INFO_STREAM_THROTTLE(
      0.5, "state: " << stateToString(current_state_) << ", commanded force: " << desired_force_
                     << "N, measured z force: "
                     << measured_wrench_O(kFreeAxisZ) - initial_wrench_O_(kFreeAxisZ)
                     << "N, free-plane offset: "
                     << (pose.translation()(kFreeAxisX) - initial_position_(kFreeAxisX)) << "m x, "
                     << (pose.translation()(kFreeAxisZ) - initial_position_(kFreeAxisZ)) << "m z");

  // Ramp the commanded force towards the target, so both the startup ramp and
  // live dynamic_reconfigure changes are smooth.
  desired_force_ = force_filter_gain_ * target_force_ + (1.0 - force_filter_gain_) * desired_force_;

  if (next_state != current_state_) {
    transitionToState(next_state, time);
  }
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

  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  wrench.tail(3) = -rotational_stiffness_ * error - rotational_damping_ * velocity.tail(3);
  return wrench;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::poseHoldWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const {
  Eigen::Matrix<double, 6, 1> wrench = orientationHoldWrench(pose, velocity);
  wrench.head(3) = -lateral_stiffness_ * (pose.translation() - initial_position_) -
                   lateral_damping_ * velocity.head(3);
  return wrench;
}

Eigen::Matrix<double, 6, 1> WeightLiftController::liftWrench(
    const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity,
    const Eigen::Matrix<double, 6, 1>& measured_wrench_O, const ros::Duration& period) {
  Eigen::Matrix<double, 6, 1> wrench = orientationHoldWrench(pose, velocity);

  // Cancel whatever was already pulling on the flange at startup (an unmodelled
  // tool weight, mostly), so the free plane is force-neutral apart from the
  // force we command. Translations only: the orientation hold covers torques.
  wrench.head(3) += initial_wrench_O_.head(3);

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

  // 3. Hold base Y so the end effector stays in the X-Z plane it started in.
  wrench(kLockedAxisY) +=
      -lateral_stiffness_ * (pose.translation()(kLockedAxisY) - initial_position_(kLockedAxisY)) -
      lateral_damping_ * velocity(kLockedAxisY);

  // 4. Light damping inside the free plane. No stiffness: the human sets the
  //    position, this only keeps the free axes from drifting and ringing.
  wrench(kFreeAxisX) += -free_damping_ * velocity(kFreeAxisX);
  wrench(kFreeAxisZ) += -free_damping_ * velocity(kFreeAxisZ);

  // 5. Virtual walls, so a release cannot run the arm into its joint limits.
  wrench(kFreeAxisX) += wallForce(pose.translation()(kFreeAxisX) - initial_position_(kFreeAxisX),
                                  velocity(kFreeAxisX), wall_x_min_, wall_x_max_);
  wrench(kFreeAxisZ) += wallForce(pose.translation()(kFreeAxisZ) - initial_position_(kFreeAxisZ),
                                  velocity(kFreeAxisZ), wall_z_min_, wall_z_max_);

  // 6. Speed cap in the free plane: brake whatever exceeds max_free_speed_.
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

Eigen::Matrix<double, 7, 1> WeightLiftController::nullspaceTorque(
    const Eigen::Matrix<double, 6, 7>& jacobian, const Eigen::Matrix<double, 7, 1>& q,
    const Eigen::Matrix<double, 7, 1>& dq) const {
  // Keeps the redundant DOF (the elbow) near its startup configuration while
  // the human drags the end effector around the free plane.
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  return (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
         (nullspace_stiffness_ * (q_d_nullspace_ - q) - 2.0 * std::sqrt(nullspace_stiffness_) * dq);
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
}

std::string WeightLiftController::stateToString(ControllerState state) const {
  switch (state) {
    case ControllerState::UNKNOWN:
      return "UNKNOWN";
    case ControllerState::INITIAL:
      return "INITIAL";
    case ControllerState::LIFT:
      return "LIFT";
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
        "WeightLiftController: initial wrench compensation disabled, weight lift "
        "active immediately");
    return ControllerState::LIFT;
  }

  // Track the estimate until it stops moving, then keep the last value as bias.
  const double change = (measured_wrench_O - initial_wrench_O_).norm();
  initial_wrench_O_ = measured_wrench_O;

  const double elapsed = (time - state_entry_time_).toSec();
  if (elapsed >= min_settle_sec_ && change < settle_wrench_tolerance_) {
    ROS_INFO_STREAM("WeightLiftController: weight lift active, initial wrench "
                    << initial_wrench_O_.head(3).transpose() << "N");
    return ControllerState::LIFT;
  }
  if (elapsed >= max_settle_sec_) {
    ROS_WARN_STREAM("WeightLiftController: external wrench estimate did not settle within "
                    << max_settle_sec_ << "s (last change " << change
                    << "N), weight lift active anyway with initial wrench "
                    << initial_wrench_O_.head(3).transpose() << "N");
    return ControllerState::LIFT;
  }

  return ControllerState::INITIAL;
}

}  // namespace franka_weight_lift

PLUGINLIB_EXPORT_CLASS(franka_weight_lift::WeightLiftController,
                       controller_interface::ControllerBase)

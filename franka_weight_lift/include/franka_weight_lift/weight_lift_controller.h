#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "franka_weight_lift/weight_lift_paramConfig.h"

namespace franka_weight_lift {

/**
 * A franka_hw effort controller that renders a constant downward force at the
 * end effector while leaving the end effector free to be moved by hand inside a
 * vertical plane.
 *
 * Task frame is the robot base frame O. Of the six end-effector DOF:
 *  - base X and base Z are free: no position servo and no damping at all, only
 *    a speed cap and, once a force has been requested, virtual walls. A human
 *    can lift, lower and slide the tool.
 *  - base Y is held by a stiff spring/damper at y = 0, so the tool works in the
 *    base's own X-Z plane.
 *  - all three rotations are held by a stiff spring/damper at diag(1, -1, -1):
 *    end-effector X along base X, end-effector Y along -base Y, end-effector Z
 *    along -base Z, i.e. the tool pointing straight down.
 *
 * Those last two are *targets*, not startup values: the ALIGN state drives the
 * arm to them right after the controller is spawned, so **the arm moves on its
 * own at startup**.
 *
 * On top of that the controller commands `desired_force` newtons along -Z of
 * the base (i.e. towards the earth, along gravity), optionally closed-loop on
 * the force actually transmitted to the environment. It defaults to 0 N, so the
 * tool can first be carried anywhere in the plane; the virtual walls anchor
 * themselves at the pose where a non-zero force is first requested.
 *
 * Sign convention (libfranka): O_F_ext_hat_K is the wrench the robot applies
 * *to* the environment, expressed in base coordinates. Pressing/pulling
 * downwards is therefore a negative z entry, both when commanding and when
 * measuring.
 */
class WeightLiftController : public controller_interface::MultiInterfaceController<
                                 franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface,
                                 hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  // Base-frame axis roles. The two free axes span the plane the human works in.
  static constexpr int kFreeAxisX = 0;
  static constexpr int kLockedAxisY = 1;
  static constexpr int kFreeAxisZ = 2;

  bool readParameters(ros::NodeHandle& node_handle);

  /** Spring/damper holding the orientation at orientation_d_. Base frame. */
  Eigen::Matrix<double, 6, 1> orientationHoldWrench(
      const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** Fully stiff 6-DOF hold at hold_position_ / orientation_d_. */
  Eigen::Matrix<double, 6, 1> poseHoldWrench(const Eigen::Isometry3d& pose,
                                             const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** The weight-lift wrench: pull-down + plane constraint + safety terms. */
  Eigen::Matrix<double, 6, 1> liftWrench(const Eigen::Isometry3d& pose,
                                         const Eigen::Matrix<double, 6, 1>& velocity,
                                         const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                                         const ros::Duration& period);

  /** Clamped spring/damper for one held translational axis. */
  double lateralHoldForce(double offset, double velocity) const;

  /** One-sided virtual wall on a free axis. Returns 0 inside [min, max]. */
  double wallForce(double offset, double velocity, double min, double max) const;

  Eigen::Matrix<double, 7, 1> nullspaceTorque(const Eigen::Matrix<double, 6, 7>& jacobian,
                                              const Eigen::Matrix<double, 7, 1>& q,
                                              const Eigen::Matrix<double, 7, 1>& dq) const;

  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  void weightLiftParamCallback(franka_weight_lift::weight_lift_paramConfig& config, uint32_t level);

 private:
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::NodeHandle dynamic_reconfigure_weight_lift_param_node_;
  std::unique_ptr<dynamic_reconfigure::Server<franka_weight_lift::weight_lift_paramConfig>>
      dynamic_server_weight_lift_param_;

 private:
  // Stiff-hold targets. Captured from the startup pose in starting() so the
  // INITIAL state stays put, then overwritten by beginAlignment() with the
  // base-aligned targets (y = 0, diag(1, -1, -1)) for ALIGN and LIFT.
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  // Nullspace reference for the elbow. Re-latched when the align move finishes,
  // so it holds the configuration the aligned pose needs rather than the one the
  // arm happened to start in.
  Eigen::Matrix<double, 7, 1> q_d_nullspace_{Eigen::Matrix<double, 7, 1>::Zero()};
  // Wrench already acting when the controller started (an unmodelled tool
  // weight, mostly). Subtracted from the measurement and added to the command
  // so the free plane is force-neutral apart from what we ask for.
  Eigen::Matrix<double, 6, 1> initial_wrench_O_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Anchor of the virtual walls, latched the moment a non-zero force is asked
  // for. While no force is asked for the walls are off and the free plane is
  // unbounded, so the tool can be carried to wherever the work happens.
  Eigen::Vector3d wall_origin_{Eigen::Vector3d::Zero()};
  bool walls_armed_{false};

  // Commanded force, low-pass filtered towards target_force_ so that both the
  // startup ramp (from 0) and live dynamic_reconfigure changes are smooth.
  double desired_force_{0.0};
  double force_integral_{0.0};

  // Live-tunable (dynamic_reconfigure), initialised from the config yaml.
  double target_force_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};

 private:
  // Parameters read from the config yaml.
  double force_filter_gain_{0.001};
  double max_force_correction_{15.0};
  double max_force_integral_{50.0};
  double integral_freeze_speed_{0.05};

  double lateral_stiffness_{1200.0};
  double lateral_damping_{55.0};
  double max_lateral_force_{30.0};
  double rotational_stiffness_{80.0};
  double rotational_damping_{14.0};
  double max_rotational_torque_{10.0};
  // Keeps the redundant DOF near q_d_nullspace_. Projected into the nullspace of
  // J^T, so it does not add end-effector force.
  double nullspace_stiffness_{15.0};

  double wall_x_min_{-0.30};
  double wall_x_max_{0.30};
  double wall_z_min_{-0.25};
  double wall_z_max_{0.10};
  double wall_stiffness_{2000.0};
  double wall_damping_{70.0};

  double max_free_speed_{0.5};
  double brake_damping_{200.0};

  bool compensate_initial_wrench_{true};
  double settle_wrench_tolerance_{0.05};
  double min_settle_sec_{1.0};
  double max_settle_sec_{5.0};

  double align_position_tolerance_{0.005};
  double align_orientation_tolerance_{0.02};
  double max_align_sec_{10.0};
  double max_align_y_offset_{0.40};
  double max_align_angle_{0.79};

  const double delta_tau_max_{1.0};
  const Eigen::Matrix<double, 7, 1> tau_max_ =
      (Eigen::Matrix<double, 7, 1>() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();

 private:
  /* State machine
   * 1. INITIAL: hold the startup pose stiffly in all six DOF until the external
   *    wrench estimate has settled, then latch it as the bias.
   * 2. ALIGN: drive base Y to 0 and the attitude to diag(1, -1, -1), holding
   *    base X and Z where they are. The arm moves on its own here.
   * 3. LIFT: render the commanded downward force, free in the base X-Z plane.
   * FAULT: alignment was refused because the startup pose is too far off the
   *    targets; hold the startup pose and wait for the operator.
   */
  enum class ControllerState { UNKNOWN, INITIAL, ALIGN, LIFT, FAULT };

  ControllerState current_state_{ControllerState::UNKNOWN};
  ControllerState previous_state_{ControllerState::UNKNOWN};
  ros::Time state_entry_time_;

  std::string stateToString(ControllerState state) const;
  void transitionToState(const ControllerState& new_state, const ros::Time& time);
  ControllerState handleInitialState(const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                                     const ros::Time& time);
  /** Checks the startup pose and swaps in the base-aligned hold targets. */
  ControllerState beginAlignment(const Eigen::Isometry3d& pose);
  ControllerState handleAlignState(const Eigen::Isometry3d& pose,
                                   const Eigen::Matrix<double, 6, 1>& velocity,
                                   const ros::Time& time);
};

}  // namespace franka_weight_lift

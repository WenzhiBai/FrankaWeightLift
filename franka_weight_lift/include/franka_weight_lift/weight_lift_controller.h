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
 * Task frame is the robot base frame O. Of the six end-effector DOF only two are
 * held; everything else is free, with no stiffness and no damping at all:
 *  - base X and base Z translation are free. A speed cap and, once a force has
 *    been requested, virtual walls are the only terms acting there, and both are
 *    zero until they trigger. This is the plane the human works in.
 *  - base Y translation is held at y = 0, so the tool works in the base's own
 *    X-Z plane.
 *  - the tool axis (end-effector Z) is held *in* that plane: the one held
 *    rotational DOF is the sideways tip out of the base X-Z plane. Tilting the
 *    tool within the plane and spinning it while it hangs vertically are both
 *    free, and cost exactly nothing.
 *
 * Those two targets are reached by an ALIGN state right after the controller is
 * spawned, so **the arm moves on its own at startup**. Alignment removes the y
 * offset and the sideways tip only; whatever tilt and spin the arm was parked
 * with are left alone.
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

  /**
   * The one held rotational DOF. `angle` is the signed angle by which the tool
   * axis (end-effector Z) has tipped out of the base X-Z plane, and `axis` is
   * the unit base-frame direction a torque has to act along to change it - a
   * combination of base X and base Z in general, and never base Y, which is why
   * tilting within the plane is always free. `axis` is exactly zero in the
   * singular case, where the tool axis points along base Y and no torque can
   * bring it back to the plane.
   */
  struct ToolAxisTip {
    double angle{0.0};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
  };
  ToolAxisTip toolAxisTip(const Eigen::Isometry3d& pose) const;

  /** Spring/damper on that one DOF. Zero torque in the two free directions. */
  Eigen::Matrix<double, 6, 1> toolAxisHoldWrench(const Eigen::Isometry3d& pose,
                                                 const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** Stiff spring/damper on all three translations, at hold_position_. */
  Eigen::Vector3d translationHoldForce(const Eigen::Isometry3d& pose,
                                       const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** Full 6-DOF stiff hold at hold_position_ / orientation_d_, for INITIAL. */
  Eigen::Matrix<double, 6, 1> poseHoldWrench(const Eigen::Isometry3d& pose,
                                             const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** Stiff translations (base Y on its aligned target) + the tool-axis hold. */
  Eigen::Matrix<double, 6, 1> alignWrench(const Eigen::Isometry3d& pose,
                                          const Eigen::Matrix<double, 6, 1>& velocity) const;

  /** The weight-lift wrench: pull-down + plane constraint + safety terms. */
  Eigen::Matrix<double, 6, 1> liftWrench(const Eigen::Isometry3d& pose,
                                         const Eigen::Matrix<double, 6, 1>& velocity,
                                         const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                                         const ros::Duration& period);

  /** Spring/damper holding the full attitude at orientation_d_. Base frame. */
  Eigen::Matrix<double, 6, 1> orientationHoldWrench(
      const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 1>& velocity) const;

  /**
   * Averages the external wrench estimate until it has held still for
   * min_settle_sec_, then latches the mean as initial_wrench_O_. Any excursion
   * from the running mean larger than settle_wrench_tolerance_ restarts the
   * window, so the mean is always taken over a stretch where nothing moved.
   * Returns true once the bias has been latched.
   */
  bool updateWrenchBias(const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                        const ros::Time& time);

  /** Clamped spring/damper for one held translational axis. */
  double lateralHoldForce(double offset, double velocity) const;

  /** One-sided virtual wall on a free axis. Returns 0 inside [min, max]. */
  double wallForce(double offset, double velocity, double min, double max) const;

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
  // Translational hold target. Captured from the startup pose in starting() so
  // the INITIAL state stays put; beginAlignment() then sets its Y entry to 0.
  // The X and Z entries only ever hold the arm still during INITIAL and ALIGN -
  // in LIFT those two axes are free and the human sets them.
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  // Startup attitude, used only by the full 6-DOF hold of INITIAL and FAULT.
  // From ALIGN on, the tool-axis constraint replaces it and the tool's tilt and
  // spin are free, so there is no attitude target any more.
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  // Wrench already acting when the controller started (an unmodelled tool
  // weight, mostly). Subtracted from the measurement and added to the command
  // so the free DOF are force-neutral apart from what we ask for. The torque
  // part matters as soon as ALIGN frees the tool's tilt and spin: without it an
  // off-axis tool CoM would simply turn the tool over. It is only exact at the
  // attitude where it was measured - a correct setLoad is the real fix.
  Eigen::Matrix<double, 6, 1> initial_wrench_O_{Eigen::Matrix<double, 6, 1>::Zero()};
  // Accumulator updateWrenchBias() averages over. bias_samples_ == 0 means the
  // window has not started yet, which is how INITIAL and CALIBRATE reset it.
  Eigen::Matrix<double, 6, 1> bias_sum_{Eigen::Matrix<double, 6, 1>::Zero()};
  int bias_samples_{0};
  ros::Time bias_window_start_;

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
  // Operator request to re-measure the wrench bias at the current pose, edge
  // triggered on the reconfigure checkbox going false -> true.
  bool recalibrate_requested_{false};
  bool last_recalibrate_{false};

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

  double wall_x_min_{-0.30};
  double wall_x_max_{0.30};
  double wall_z_min_{-0.25};
  double wall_z_max_{0.10};
  double wall_stiffness_{2000.0};
  double wall_damping_{70.0};

  double max_free_speed_{0.5};
  double brake_damping_{200.0};

  bool compensate_initial_wrench_{true};
  double settle_wrench_tolerance_{0.2};
  double min_settle_sec_{1.0};
  double max_settle_sec_{5.0};

  double align_position_tolerance_{0.005};
  double align_orientation_tolerance_{0.02};
  double max_align_sec_{10.0};
  double max_align_y_offset_{0.25};
  double max_align_angle_{1.0};

  const double delta_tau_max_{1.0};
  const Eigen::Matrix<double, 7, 1> tau_max_ =
      (Eigen::Matrix<double, 7, 1>() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();

 private:
  /* State machine
   * 1. INITIAL: hold the startup pose stiffly in all six DOF until the external
   *    wrench estimate has settled, then latch it as the bias.
   * 2. ALIGN: drive base Y to 0 and the tool axis into the base X-Z plane,
   *    holding base X and Z where they are and leaving the tool's tilt and spin
   *    alone. The arm moves on its own here.
   * 3. LIFT: render the commanded downward force, free in the base X-Z plane.
   * CALIBRATE: re-measure the wrench bias at the pose the tool is in now, on
   *    operator request, holding that pose stiffly while it does. Returns to
   *    LIFT. This is the accurate way to zero the force: the bias latched at
   *    spawn only holds at the configuration it was measured in.
   * FAULT: alignment was refused because the startup pose is too far off the
   *    targets; hold the startup pose and wait for the operator.
   */
  enum class ControllerState { UNKNOWN, INITIAL, ALIGN, LIFT, CALIBRATE, FAULT };

  ControllerState current_state_{ControllerState::UNKNOWN};
  ControllerState previous_state_{ControllerState::UNKNOWN};
  ros::Time state_entry_time_;

  std::string stateToString(ControllerState state) const;
  void transitionToState(const ControllerState& new_state, const ros::Time& time);
  ControllerState handleInitialState(const Eigen::Matrix<double, 6, 1>& measured_wrench_O,
                                     const ros::Time& time);
  /** Checks the startup pose and moves the base Y hold target onto 0. */
  ControllerState beginAlignment(const Eigen::Isometry3d& pose);
  ControllerState handleAlignState(const Eigen::Isometry3d& pose,
                                   const Eigen::Matrix<double, 6, 1>& velocity,
                                   const ros::Time& time);
  /** Checks the operator's recalibration request and holds the current pose. */
  ControllerState beginCalibration(const Eigen::Isometry3d& pose,
                                   const Eigen::Matrix<double, 6, 1>& velocity);
};

}  // namespace franka_weight_lift

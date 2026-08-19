#ifndef PX4CTRL_OMMPC_CONTROLLER_H
#define PX4CTRL_OMMPC_CONTROLLER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_utils/PolyTraj.h>

#include <memory>
#include <string>

class Parameter_t;

namespace px4ctrl_ommpc
{

struct ControlOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  double collective_acceleration{0.0};
};

struct DebugOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool reference_valid{false};
  bool solver_success{false};
  int trajectory_id{0};
  double trajectory_time{0.0};
  double solve_time_ms{0.0};
  double optimal_cost{0.0};

  Eigen::Vector3d reference_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d reference_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond reference_attitude{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d attitude_error{Eigen::Vector3d::Zero()};
  double reference_collective_acceleration{0.0};
  double commanded_collective_acceleration{0.0};
  Eigen::Vector3d reference_bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Vector3d commanded_bodyrates{Eigen::Vector3d::Zero()};
};

/**
 * On-manifold MPC core used by px4ctrl's CMD_CTRL state.
 *
 * The class owns the native PolyTraj reference and deliberately has no
 * takeoff/landing/hover reference generators. Those states remain under
 * px4ctrl algorithm 0.
 */
class OnManifoldMPC
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OnManifoldMPC();
  ~OnManifoldMPC();

  OnManifoldMPC(const OnManifoldMPC &) = delete;
  OnManifoldMPC &operator=(const OnManifoldMPC &) = delete;

  bool initialize(const Parameter_t &param);

  void feedTrajectory(const traj_utils::PolyTrajConstPtr &msg);
  void feedHeartbeat(const std_msgs::EmptyConstPtr &msg);

  bool trajectoryReady(const ros::Time &now) const;
  bool trajectoryFinished(const ros::Time &now) const;
  bool heartbeatTimedOut(const ros::Time &now) const;
  bool getTrajectoryEndPosition(Eigen::Vector3d &position) const;
  void invalidateTrajectory(const std::string &reason);

  bool update(const Eigen::Vector3d &position,
              const Eigen::Vector3d &velocity,
              const Eigen::Quaterniond &attitude,
              const ros::Time &now,
              ControlOutput &output);

  const std::string &lastError() const;
  double timingMilliseconds() const;
  const DebugOutput &debugOutput() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace px4ctrl_ommpc

#endif

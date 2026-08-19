#include <gtest/gtest.h>

#include "PX4CtrlParam.h"
#include "controller.h"
#include "ommpc_controller.h"

#include <iomanip>
#include <iostream>
#include <vector>

namespace
{

Parameter_t makeParameters(const bool use_heartbeat)
{
  Parameter_t parameter;
  parameter.pose_solver = 3;
  parameter.mass = 1.0;
  parameter.gra = 9.81;
  parameter.max_angle = -1.0;
  parameter.ctrl_freq_max = 200.0;
  parameter.max_manual_vel = 1.0;
  parameter.low_voltage = 21.0;

  parameter.gain.Kp0 = 4.0;
  parameter.gain.Kp1 = 4.0;
  parameter.gain.Kp2 = 4.0;
  parameter.gain.Kv0 = 3.5;
  parameter.gain.Kv1 = 3.5;
  parameter.gain.Kv2 = 3.5;
  parameter.gain.Kvi0 = 0.0;
  parameter.gain.Kvi1 = 0.0;
  parameter.gain.Kvi2 = 0.0;
  parameter.gain.Kvd0 = 0.0;
  parameter.gain.Kvd1 = 0.0;
  parameter.gain.Kvd2 = 0.0;
  parameter.gain.KAngR = 20.0;
  parameter.gain.KAngP = 20.0;
  parameter.gain.KAngY = 4.0;

  parameter.rt_drag.x = 0.0;
  parameter.rt_drag.y = 0.0;
  parameter.rt_drag.z = 0.0;
  parameter.rt_drag.k_thrust_horz = 0.0;
  parameter.thr_map.print_val = false;
  parameter.thr_map.K1 = 0.7583;
  parameter.thr_map.K2 = 1.6942;
  parameter.thr_map.K3 = 0.6786;
  parameter.thr_map.accurate_thrust_model = false;
  parameter.thr_map.hover_percentage = 0.45;
  parameter.thr_map.noisy_imu = false;

  parameter.ommpc.step_T = 0.01;
  parameter.ommpc.solve_frequency = 100.0;
  parameter.ommpc.Q_pos_xy = 1000.0;
  parameter.ommpc.Q_pos_z = 800.0;
  parameter.ommpc.Q_velocity = 20.0;
  parameter.ommpc.Q_attitude_rp = 40.0;
  parameter.ommpc.Q_attitude_yaw = 40.0;
  parameter.ommpc.R_thrust = 0.5;
  parameter.ommpc.R_pitchroll = 1.2;
  parameter.ommpc.R_yaw = 0.6;
  parameter.ommpc.state_cost_exponential = 0.5;
  parameter.ommpc.input_cost_exponential = 0.5;
  parameter.ommpc.max_bodyrate_xy = 6.0;
  parameter.ommpc.max_bodyrate_z = 4.0;
  parameter.ommpc.min_thrust = 1.0;
  parameter.ommpc.max_thrust = 30.0;
  parameter.ommpc.use_fix_yaw = true;
  parameter.ommpc.use_trajectory_ending_pos = true;
  parameter.ommpc.use_heartbeat = use_heartbeat;
  parameter.ommpc.heartbeat_timeout = 0.5;
  return parameter;
}

traj_utils::PolyTrajPtr makeKinematicTrajectory(
    const ros::Time &start_time,
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &acceleration,
    const Eigen::Vector3d &jerk,
    const int trajectory_id = 1,
    const double duration = 1.0)
{
  traj_utils::PolyTrajPtr message(new traj_utils::PolyTraj());
  message->drone_id = 0;
  message->traj_id = trajectory_id;
  message->start_time = start_time;
  message->order = 5;
  message->duration.push_back(duration);
  message->coef_x.assign(6, 0.0);
  message->coef_y.assign(6, 0.0);
  message->coef_z.assign(6, 0.0);
  for (int axis = 0; axis < 3; ++axis)
  {
    std::vector<float> *coefficients = axis == 0 ? &message->coef_x :
                                       axis == 1 ? &message->coef_y :
                                                   &message->coef_z;
    (*coefficients)[2] = jerk(axis) / 6.0;
    (*coefficients)[3] = acceleration(axis) / 2.0;
    (*coefficients)[4] = velocity(axis);
    (*coefficients)[5] = position(axis);
  }
  return message;
}

struct ComparisonResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double ommpc_collective_acceleration{0.0};
  double alg0_collective_acceleration{0.0};
  Eigen::Vector3d ommpc_bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Vector3d alg0_bodyrates{Eigen::Vector3d::Zero()};
};

ComparisonResult compareControllers(
    const Eigen::Vector3d &reference_position,
    const Eigen::Vector3d &reference_velocity,
    const Eigen::Vector3d &reference_acceleration,
    const Eigen::Vector3d &reference_jerk,
    const Eigen::Vector3d &estimated_position,
    const Eigen::Vector3d &estimated_velocity,
    const Eigen::Quaterniond &estimated_attitude)
{
  Parameter_t parameter = makeParameters(false);
  const ros::Time start_time(10.0);
  const traj_utils::PolyTrajPtr trajectory = makeKinematicTrajectory(
      start_time, reference_position, reference_velocity,
      reference_acceleration, reference_jerk);

  px4ctrl_ommpc::OnManifoldMPC ommpc;
  EXPECT_TRUE(ommpc.initialize(parameter));
  ommpc.feedTrajectory(trajectory);
  px4ctrl_ommpc::ControlOutput ommpc_output;
  EXPECT_TRUE(ommpc.update(estimated_position, estimated_velocity,
                           estimated_attitude, start_time, ommpc_output))
      << ommpc.lastError();

  Controller alg0(parameter);
  Desired_State_t desired;
  desired.p = reference_position;
  desired.v = reference_velocity;
  desired.a = reference_acceleration;
  desired.j = reference_jerk;
  desired.yaw = 0.0;
  desired.yaw_rate = 0.0;
  desired.q = Eigen::Quaterniond::Identity();

  Odom_Data_t odometry;
  odometry.p = estimated_position;
  odometry.v = estimated_velocity;
  odometry.q = estimated_attitude;
  odometry.w.setZero();
  Imu_Data_t imu;
  imu.q = estimated_attitude;
  imu.w.setZero();
  imu.a.setZero();
  Controller_Output_t alg0_output;
  const quadrotor_msgs::Px4ctrlDebug alg0_debug =
      alg0.update_alg0(desired, odometry, imu, alg0_output, 24.0);

  ComparisonResult result;
  result.ommpc_collective_acceleration = ommpc_output.collective_acceleration;
  result.alg0_collective_acceleration = alg0_debug.des_thr;
  result.ommpc_bodyrates = ommpc_output.bodyrates;
  result.alg0_bodyrates = alg0_output.bodyrates;
  return result;
}

void printComparison(const std::string &name, const ComparisonResult &result)
{
  std::cout << std::fixed << std::setprecision(5)
            << "[OMMPC_VS_ALG0] " << name
            << " collective_accel=(" << result.ommpc_collective_acceleration
            << ", " << result.alg0_collective_acceleration << ")"
            << " bodyrates_ommpc=[" << result.ommpc_bodyrates.transpose() << "]"
            << " bodyrates_alg0=[" << result.alg0_bodyrates.transpose() << "]"
            << " bodyrate_error="
            << (result.ommpc_bodyrates - result.alg0_bodyrates).norm()
            << std::endl;
}

struct SimulatedState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
};

void integrateIdealQuadrotor(SimulatedState &state,
                             const double collective_acceleration,
                             const Eigen::Vector3d &bodyrates,
                             const double gravity,
                             const double time_step)
{
  const Eigen::Vector3d acceleration =
      state.attitude * Eigen::Vector3d::UnitZ() * collective_acceleration -
      gravity * Eigen::Vector3d::UnitZ();
  state.position += state.velocity * time_step +
                    0.5 * acceleration * time_step * time_step;
  state.velocity += acceleration * time_step;

  const double rotation_angle = bodyrates.norm() * time_step;
  if (rotation_angle > 1.0e-12)
  {
    state.attitude =
        (state.attitude * Eigen::Quaterniond(
                              Eigen::AngleAxisd(rotation_angle,
                                                bodyrates.normalized())))
            .normalized();
  }
}

traj_utils::PolyTrajPtr makeHoverTrajectory(const ros::Time &start_time,
                                            const double altitude = 1.0,
                                            const int trajectory_id = 1,
                                            const double duration = 1.0)
{
  traj_utils::PolyTrajPtr message(new traj_utils::PolyTraj());
  message->drone_id = 0;
  message->traj_id = trajectory_id;
  message->start_time = start_time;
  message->order = 5;
  message->duration.push_back(duration);
  message->coef_x.assign(6, 0.0);
  message->coef_y.assign(6, 0.0);
  message->coef_z.assign(6, 0.0);
  // Coefficients are ordered from t^5 to t^0.
  message->coef_z[5] = altitude;
  return message;
}

TEST(OnManifoldMPC, RejectsMalformedCoefficientArrays)
{
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(false)));

  auto message = makeHoverTrajectory(ros::Time(10.0));
  message->coef_x.pop_back();
  controller.feedTrajectory(message);
  EXPECT_FALSE(controller.trajectoryReady(ros::Time(10.1)));
  EXPECT_NE(controller.lastError().find("coefficient length mismatch"), std::string::npos);
}

TEST(OnManifoldMPC, SolvesZeroErrorHoverTrajectory)
{
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(false)));
  controller.feedTrajectory(makeHoverTrajectory(ros::Time(10.0)));
  ASSERT_TRUE(controller.trajectoryReady(ros::Time(10.1)));

  px4ctrl_ommpc::ControlOutput output;
  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                ros::Time(10.1), output))
      << controller.lastError();
  EXPECT_NEAR(output.collective_acceleration, 9.81, 1.0e-3);
  EXPECT_NEAR(output.bodyrates.norm(), 0.0, 1.0e-3);

  const px4ctrl_ommpc::DebugOutput &debug = controller.debugOutput();
  EXPECT_TRUE(debug.reference_valid);
  EXPECT_TRUE(debug.solver_success);
  EXPECT_EQ(debug.trajectory_id, 1);
  EXPECT_NEAR(debug.trajectory_time, 0.1, 1.0e-9);
  EXPECT_NEAR(debug.reference_position.z(), 1.0, 1.0e-9);
  EXPECT_NEAR(debug.commanded_collective_acceleration, 9.81, 1.0e-3);
  EXPECT_NEAR(debug.commanded_bodyrates.norm(), 0.0, 1.0e-3);

  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                ros::Time(10.105), output));
  EXPECT_TRUE(controller.debugOutput().solver_success);
  EXPECT_NEAR(controller.debugOutput().solve_time_ms, 0.0, 1.0e-12);

  Eigen::Vector3d endpoint;
  ASSERT_TRUE(controller.getTrajectoryEndPosition(endpoint));
  EXPECT_NEAR((endpoint - Eigen::Vector3d(0.0, 0.0, 1.0)).norm(), 0.0, 1.0e-9);
}

TEST(OnManifoldMPC, EvaluatesPolyTrajAccelerationInDescendingCoefficientOrder)
{
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(false)));
  auto trajectory = makeHoverTrajectory(ros::Time(10.0));
  // z(t) = 1 + t^2, so the commanded vertical acceleration is 2 m/s^2.
  trajectory->coef_z[3] = 1.0;
  controller.feedTrajectory(trajectory);

  px4ctrl_ommpc::ControlOutput output;
  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                ros::Time(10.0), output))
      << controller.lastError();
  EXPECT_NEAR(output.collective_acceleration, 11.81, 1.0e-3);
  EXPECT_NEAR(output.bodyrates.norm(), 0.0, 1.0e-3);
}

TEST(OnManifoldMPC, CommandsPositivePitchTowardPositivePositionError)
{
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(false)));
  auto trajectory = makeHoverTrajectory(ros::Time(10.0));
  trajectory->coef_x[5] = 1.0;
  controller.feedTrajectory(trajectory);

  px4ctrl_ommpc::ControlOutput output;
  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                ros::Time(10.1), output))
      << controller.lastError();
  EXPECT_GT(output.bodyrates.y(), 1.0e-4);
  EXPECT_GT(controller.debugOutput().commanded_bodyrates.y(), 1.0e-4);
  EXPECT_GT(controller.debugOutput().position_error.x(), 0.0);
}

TEST(OnManifoldMPC, RequiresFreshHeartbeatWhenEnabled)
{
  ros::Time::init();
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(true)));
  const ros::Time now = ros::Time::now();
  controller.feedTrajectory(makeHoverTrajectory(now));
  EXPECT_FALSE(controller.trajectoryReady(now));

  std_msgs::EmptyConstPtr heartbeat(new std_msgs::Empty());
  controller.feedHeartbeat(heartbeat);
  const ros::Time heartbeat_time = ros::Time::now();
  EXPECT_TRUE(controller.trajectoryReady(heartbeat_time));
  EXPECT_TRUE(controller.heartbeatTimedOut(heartbeat_time + ros::Duration(0.6)));
}

TEST(OnManifoldMPC, KeepsCurrentSegmentUntilFutureReplanStarts)
{
  ros::Time::init();
  px4ctrl_ommpc::OnManifoldMPC controller;
  ASSERT_TRUE(controller.initialize(makeParameters(false)));

  const ros::Time receipt_time = ros::Time::now();
  controller.feedTrajectory(
      makeHoverTrajectory(receipt_time - ros::Duration(0.1), 1.0, 1, 0.2));
  controller.feedTrajectory(
      makeHoverTrajectory(receipt_time + ros::Duration(1.0), 2.0, 2, 1.0));

  const ros::Time gap_time = receipt_time + ros::Duration(0.5);
  ASSERT_TRUE(controller.trajectoryReady(gap_time));
  px4ctrl_ommpc::ControlOutput output;
  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(), gap_time, output))
      << controller.lastError();
  EXPECT_NEAR(output.collective_acceleration, 9.81, 1.0e-3);

  const ros::Time future_time = receipt_time + ros::Duration(1.1);
  ASSERT_TRUE(controller.trajectoryReady(future_time));
  ASSERT_TRUE(controller.update(Eigen::Vector3d(0.0, 0.0, 2.0),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(), future_time, output))
      << controller.lastError();
  Eigen::Vector3d endpoint;
  ASSERT_TRUE(controller.getTrajectoryEndPosition(endpoint));
  EXPECT_NEAR(endpoint.z(), 2.0, 1.0e-9);
}

TEST(OnManifoldMPC, FeedforwardNumericallyMatchesAlgorithm0)
{
  struct Scenario
  {
    std::string name;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d jerk;
  };
  const std::vector<Scenario> scenarios = {
      {"hover", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()},
      {"vertical_acceleration", Eigen::Vector3d(0.0, 0.0, 0.4),
       Eigen::Vector3d(0.0, 0.0, 1.2), Eigen::Vector3d(0.0, 0.0, -0.3)},
      {"coupled_acceleration_and_jerk", Eigen::Vector3d(0.6, -0.2, 0.1),
       Eigen::Vector3d(1.2, -0.8, 0.4), Eigen::Vector3d(0.3, 0.2, -0.1)}};

  Parameter_t parameter = makeParameters(false);
  Controller flatness_model(parameter);
  for (const Scenario &scenario : scenarios)
  {
    Eigen::Quaterniond reference_attitude;
    Eigen::Vector3d reference_bodyrates;
    double reference_thrust = 0.0;
    flatness_model.minimumSingularityFlatWithDrag(
        parameter.mass, parameter.gra, scenario.velocity,
        scenario.acceleration, scenario.jerk, 0.0, 0.0,
        Eigen::Quaterniond::Identity(), reference_attitude,
        reference_bodyrates, reference_thrust);

    const ComparisonResult result = compareControllers(
        Eigen::Vector3d(0.3, -0.4, 1.2), scenario.velocity,
        scenario.acceleration, scenario.jerk,
        Eigen::Vector3d(0.3, -0.4, 1.2), scenario.velocity,
        reference_attitude);
    printComparison(scenario.name, result);
    SCOPED_TRACE(scenario.name);
    EXPECT_NEAR(result.ommpc_collective_acceleration,
                result.alg0_collective_acceleration, 2.0e-4);
    EXPECT_LT((result.ommpc_bodyrates - result.alg0_bodyrates).norm(), 2.0e-4);
  }
}

TEST(OnManifoldMPC, FeedbackHasSameDirectionAndOrderOfMagnitudeAsAlgorithm0)
{
  struct Scenario
  {
    std::string name;
    Eigen::Vector3d estimated_position;
    Eigen::Vector3d estimated_velocity;
    Eigen::Quaterniond estimated_attitude;
    bool expect_bodyrate_correction;
  };
  const std::vector<Scenario> scenarios = {
      {"positive_x_position_error", Eigen::Vector3d(-0.15, 0.0, 1.0),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), true},
      {"positive_x_velocity_error", Eigen::Vector3d(0.0, 0.0, 1.0),
       Eigen::Vector3d(-0.20, 0.0, 0.0), Eigen::Quaterniond::Identity(), true},
      {"positive_z_position_error", Eigen::Vector3d(0.0, 0.0, 0.85),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), false},
      {"positive_roll_attitude_error", Eigen::Vector3d(0.0, 0.0, 1.0),
       Eigen::Vector3d::Zero(),
       Eigen::Quaterniond(Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitX())),
       true}};

  for (const Scenario &scenario : scenarios)
  {
    const ComparisonResult result = compareControllers(
        Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        scenario.estimated_position, scenario.estimated_velocity,
        scenario.estimated_attitude);
    printComparison(scenario.name, result);
    SCOPED_TRACE(scenario.name);

    EXPECT_NEAR(result.ommpc_collective_acceleration,
                result.alg0_collective_acceleration, 1.0);
    if (scenario.expect_bodyrate_correction)
    {
      ASSERT_GT(result.alg0_bodyrates.norm(), 1.0e-6);
      const double feedback_ratio =
          result.ommpc_bodyrates.norm() / result.alg0_bodyrates.norm();
      // OM-MPC spreads its correction over a 0.2 s horizon, while alg0 applies
      // a high-gain cascaded correction immediately. With the flight defaults,
      // OM-MPC is deliberately more conservative but must retain the same sign
      // and remain within one order of magnitude of alg0.
      EXPECT_GT(result.ommpc_bodyrates.dot(result.alg0_bodyrates), 0.0);
      EXPECT_GE(feedback_ratio, 0.15);
      EXPECT_LE(feedback_ratio, 1.10);
    }
    else
    {
      EXPECT_NEAR(result.ommpc_bodyrates.norm(), 0.0, 1.0e-6);
      EXPECT_NEAR(result.alg0_bodyrates.norm(), 0.0, 1.0e-6);
    }
  }
}

TEST(OnManifoldMPC, IdealClosedLoopHoverRecoveryIsComparableToAlgorithm0)
{
  Parameter_t parameter = makeParameters(false);
  const double time_step = parameter.ommpc.step_T;
  const int simulation_steps = 200;
  const Eigen::Vector3d reference_position(0.0, 0.0, 1.0);
  const ros::Time start_time(10.0);

  SimulatedState ommpc_state;
  ommpc_state.position = Eigen::Vector3d(-0.15, 0.0, 0.85);
  SimulatedState alg0_state = ommpc_state;

  px4ctrl_ommpc::OnManifoldMPC ommpc;
  ASSERT_TRUE(ommpc.initialize(parameter));
  ommpc.feedTrajectory(makeHoverTrajectory(start_time, 1.0, 1, 3.0));

  Controller alg0(parameter);
  Desired_State_t desired;
  desired.p = reference_position;
  desired.v.setZero();
  desired.a.setZero();
  desired.j.setZero();
  desired.q = Eigen::Quaterniond::Identity();
  desired.yaw = 0.0;
  desired.yaw_rate = 0.0;

  double ommpc_squared_error_sum = 0.0;
  double alg0_squared_error_sum = 0.0;
  for (int step = 0; step < simulation_steps; ++step)
  {
    const ros::Time now = start_time + ros::Duration(step * time_step);
    ommpc_squared_error_sum +=
        (reference_position - ommpc_state.position).squaredNorm();
    alg0_squared_error_sum +=
        (reference_position - alg0_state.position).squaredNorm();

    px4ctrl_ommpc::ControlOutput ommpc_output;
    ASSERT_TRUE(ommpc.update(ommpc_state.position, ommpc_state.velocity,
                             ommpc_state.attitude, now, ommpc_output))
        << ommpc.lastError();
    integrateIdealQuadrotor(ommpc_state, ommpc_output.collective_acceleration,
                            ommpc_output.bodyrates, parameter.gra, time_step);

    Odom_Data_t odometry;
    odometry.p = alg0_state.position;
    odometry.v = alg0_state.velocity;
    odometry.q = alg0_state.attitude;
    odometry.w.setZero();
    Imu_Data_t imu;
    imu.q = alg0_state.attitude;
    imu.w.setZero();
    imu.a.setZero();
    Controller_Output_t alg0_output;
    // Exercise alg0's control law without coupling this deterministic dynamics
    // test to wall-clock time in its angular-acceleration limiter.
    alg0.last_ctrl_timestamp_ = ros::Time(0);
    alg0.update_alg0(desired, odometry, imu, alg0_output, 24.0);
    const double alg0_collective_acceleration = alg0_output.thrust * alg0.thr2acc;
    integrateIdealQuadrotor(alg0_state, alg0_collective_acceleration,
                            alg0_output.bodyrates, parameter.gra, time_step);
  }

  const double ommpc_final_error =
      (reference_position - ommpc_state.position).norm();
  const double alg0_final_error =
      (reference_position - alg0_state.position).norm();
  const double ommpc_rmse =
      std::sqrt(ommpc_squared_error_sum / simulation_steps);
  const double alg0_rmse =
      std::sqrt(alg0_squared_error_sum / simulation_steps);
  std::cout << std::fixed << std::setprecision(5)
            << "[OMMPC_VS_ALG0] ideal_hover_recovery final_error=("
            << ommpc_final_error << ", " << alg0_final_error << ") rmse=("
            << ommpc_rmse << ", " << alg0_rmse << ")" << std::endl;

  EXPECT_LT(ommpc_final_error, 0.05);
  EXPECT_LT(alg0_final_error, 0.05);
  EXPECT_LT(ommpc_rmse, 2.0 * alg0_rmse);
  EXPECT_LT(alg0_rmse, 2.0 * ommpc_rmse);
}

} // namespace

int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

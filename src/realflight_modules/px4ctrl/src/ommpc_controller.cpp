#include "ommpc_controller.h"

#include "PX4CtrlParam.h"

#include <Eigen/Sparse>
#include <osqp/osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace px4ctrl_ommpc
{
namespace
{

constexpr int kHorizonSteps = 20;
constexpr int kErrorStateDim = 9;
constexpr int kStateDim = 10;
constexpr int kInputDim = 4;
constexpr int kMaxPolynomialOrder = 10;
constexpr double kMinTrajectoryDuration = 1.0e-4;
constexpr double kMinCollectiveAcceleration = 1.0e-3;
constexpr double kHopfSingularityTolerance = 1.0e-6;

bool finiteScalar(const double value)
{
  return std::isfinite(value);
}

bool finiteVector(const Eigen::VectorXd &value)
{
  return value.allFinite();
}

Eigen::Matrix3d hat(const Eigen::Vector3d &vector)
{
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
      vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return result;
}

Eigen::Vector3d vee(const Eigen::Matrix3d &matrix)
{
  return Eigen::Vector3d(matrix(2, 1), matrix(0, 2), matrix(1, 0));
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d &rotation_vector)
{
  const double angle = rotation_vector.norm();
  const Eigen::Matrix3d skew = hat(rotation_vector);
  if (angle < 1.0e-8)
  {
    return Eigen::Matrix3d::Identity() + skew + 0.5 * skew * skew;
  }

  const double angle_squared = angle * angle;
  return Eigen::Matrix3d::Identity() +
         std::sin(angle) / angle * skew +
         (1.0 - std::cos(angle)) / angle_squared * skew * skew;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d &rotation)
{
  const double cosine = std::max(-1.0, std::min(1.0, (rotation.trace() - 1.0) * 0.5));
  const double angle = std::acos(cosine);

  if (angle < 1.0e-7)
  {
    return vee(0.5 * (rotation - rotation.transpose()));
  }

  if (M_PI - angle < 1.0e-5)
  {
    Eigen::AngleAxisd angle_axis(rotation);
    if (!finiteScalar(angle_axis.angle()) || !angle_axis.axis().allFinite())
    {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return angle_axis.angle() * angle_axis.axis();
  }

  return angle / (2.0 * std::sin(angle)) * vee(rotation - rotation.transpose());
}

Eigen::Matrix3d so3LeftJacobian(const Eigen::Vector3d &rotation_vector)
{
  const double angle = rotation_vector.norm();
  const Eigen::Matrix3d skew = hat(rotation_vector);
  if (angle < 1.0e-7)
  {
    return Eigen::Matrix3d::Identity() + 0.5 * skew + (1.0 / 6.0) * skew * skew;
  }

  const double angle_squared = angle * angle;
  return Eigen::Matrix3d::Identity() +
         (1.0 - std::cos(angle)) / angle_squared * skew +
         (angle - std::sin(angle)) / (angle_squared * angle) * skew * skew;
}

class PolynomialPiece
{
public:
  PolynomialPiece(const double duration, const Eigen::MatrixXd &coefficients)
      : duration_(duration), coefficients_(coefficients), degree_(coefficients.cols() - 1)
  {
  }

  double duration() const
  {
    return duration_;
  }

  Eigen::Vector3d evaluate(const double time, const int derivative) const
  {
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    const double clamped_time = std::max(0.0, std::min(duration_, time));

    for (int power = derivative; power <= degree_; ++power)
    {
      double factor = 1.0;
      for (int order = 0; order < derivative; ++order)
      {
        factor *= static_cast<double>(power - order);
      }
      const int column = degree_ - power;
      result += factor * std::pow(clamped_time, power - derivative) * coefficients_.col(column);
    }
    return result;
  }

private:
  double duration_;
  Eigen::MatrixXd coefficients_;
  int degree_;
};

class PolynomialTrajectory
{
public:
  void clear()
  {
    pieces_.clear();
    total_duration_ = 0.0;
  }

  void addPiece(const double duration, const Eigen::MatrixXd &coefficients)
  {
    pieces_.emplace_back(duration, coefficients);
    total_duration_ += duration;
  }

  bool empty() const
  {
    return pieces_.empty();
  }

  double totalDuration() const
  {
    return total_duration_;
  }

  Eigen::Vector3d evaluate(double time, const int derivative) const
  {
    if (pieces_.empty())
    {
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    time = std::max(0.0, std::min(total_duration_, time));
    std::size_t piece_index = 0;
    while (piece_index + 1 < pieces_.size() && time > pieces_[piece_index].duration())
    {
      time -= pieces_[piece_index].duration();
      ++piece_index;
    }
    return pieces_[piece_index].evaluate(time, derivative);
  }

  Eigen::Vector3d endPosition() const
  {
    return evaluate(total_duration_, 0);
  }

private:
  std::vector<PolynomialPiece> pieces_;
  double total_duration_{0.0};
};

struct ScheduledTrajectory
{
  PolynomialTrajectory trajectory;
  ros::Time start_time;
  ros::Time end_time;
  int id{0};
};

struct MPCSolution
{
  std::vector<Eigen::VectorXd> delta_u;
  std::vector<Eigen::VectorXd> delta_x;
  double optimal_cost{0.0};
};

class MPCWrapper
{
public:
  MPCWrapper()
      : total_variables_((kHorizonSteps + 1) * kErrorStateDim + kHorizonSteps * kInputDim),
        total_constraints_(kErrorStateDim + kHorizonSteps * kErrorStateDim +
                           kHorizonSteps * kInputDim)
  {
  }

  void buildHessian(const Eigen::Matrix<double, kErrorStateDim, 1> &state_weights,
                    const Eigen::Matrix<double, kInputDim, 1> &input_weights,
                    const double state_decay_rate,
                    const double input_decay_rate)
  {
    p_data_.resize(total_variables_);
    p_indices_.resize(total_variables_);
    p_indptr_.resize(total_variables_ + 1);
    for (int column = 0; column <= total_variables_; ++column)
    {
      p_indptr_[column] = column;
    }

    int variable = 0;
    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const double state_scale =
          std::exp(-static_cast<double>(step) / kHorizonSteps * state_decay_rate);
      for (int index = 0; index < kErrorStateDim; ++index, ++variable)
      {
        p_indices_[variable] = variable;
        p_data_[variable] = state_weights(index) * state_scale;
      }

      const double input_scale =
          std::exp(-static_cast<double>(step) / kHorizonSteps * input_decay_rate);
      for (int index = 0; index < kInputDim; ++index, ++variable)
      {
        p_indices_[variable] = variable;
        p_data_[variable] = input_weights(index) * input_scale;
      }
    }

    const double terminal_scale = std::exp(-state_decay_rate);
    for (int index = 0; index < kErrorStateDim; ++index, ++variable)
    {
      p_indices_[variable] = variable;
      p_data_[variable] = state_weights(index) * terminal_scale;
    }
  }

  void buildConstraints(const std::vector<Eigen::SparseMatrix<double>> &state_matrices,
                        const std::vector<Eigen::SparseMatrix<double>> &input_matrices,
                        const std::vector<Eigen::Matrix<double, kInputDim, 1>> &lower_input,
                        const std::vector<Eigen::Matrix<double, kInputDim, 1>> &upper_input)
  {
    int nonzeros = kErrorStateDim + kHorizonSteps * kInputDim;
    for (int step = 0; step < kHorizonSteps; ++step)
    {
      nonzeros += state_matrices[step].nonZeros() + input_matrices[step].nonZeros() +
                  kErrorStateDim;
    }

    std::vector<int> column_nonzeros(total_variables_, 0);
    for (int index = 0; index < kErrorStateDim; ++index)
    {
      ++column_nonzeros[index];
    }

    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const int state_offset = step * (kErrorStateDim + kInputDim);
      const int input_offset = state_offset + kErrorStateDim;
      const int next_state_offset = (step + 1) * (kErrorStateDim + kInputDim);

      for (int outer = 0; outer < state_matrices[step].outerSize(); ++outer)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(state_matrices[step], outer); it; ++it)
        {
          ++column_nonzeros[state_offset + it.col()];
        }
      }
      for (int outer = 0; outer < input_matrices[step].outerSize(); ++outer)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(input_matrices[step], outer); it; ++it)
        {
          ++column_nonzeros[input_offset + it.col()];
        }
      }
      for (int index = 0; index < kErrorStateDim; ++index)
      {
        ++column_nonzeros[next_state_offset + index];
      }
      for (int index = 0; index < kInputDim; ++index)
      {
        ++column_nonzeros[input_offset + index];
      }
    }

    a_indptr_.resize(total_variables_ + 1);
    a_indptr_[0] = 0;
    for (int column = 0; column < total_variables_; ++column)
    {
      a_indptr_[column + 1] = a_indptr_[column] + column_nonzeros[column];
    }

    a_data_.assign(nonzeros, 0.0);
    a_indices_.assign(nonzeros, 0);
    std::vector<int> column_position(total_variables_, 0);

    auto add_entry = [&](const int row, const int column, const double value) {
      const int position = a_indptr_[column] + column_position[column]++;
      a_data_[position] = value;
      a_indices_[position] = row;
    };

    for (int index = 0; index < kErrorStateDim; ++index)
    {
      add_entry(index, index, 1.0);
    }

    int constraint_offset = kErrorStateDim;
    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const int state_offset = step * (kErrorStateDim + kInputDim);
      const int input_offset = state_offset + kErrorStateDim;
      const int next_state_offset = (step + 1) * (kErrorStateDim + kInputDim);

      for (int outer = 0; outer < state_matrices[step].outerSize(); ++outer)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(state_matrices[step], outer); it; ++it)
        {
          add_entry(constraint_offset + it.row(), state_offset + it.col(), -it.value());
        }
      }
      for (int outer = 0; outer < input_matrices[step].outerSize(); ++outer)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(input_matrices[step], outer); it; ++it)
        {
          add_entry(constraint_offset + it.row(), input_offset + it.col(), -it.value());
        }
      }
      for (int index = 0; index < kErrorStateDim; ++index)
      {
        add_entry(constraint_offset + index, next_state_offset + index, 1.0);
      }
      constraint_offset += kErrorStateDim;
    }

    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const int input_offset = step * (kErrorStateDim + kInputDim) + kErrorStateDim;
      for (int index = 0; index < kInputDim; ++index)
      {
        add_entry(constraint_offset++, input_offset + index, 1.0);
      }
    }

    q_.assign(total_variables_, 0.0);
    lower_.assign(total_constraints_, 0.0);
    upper_.assign(total_constraints_, 0.0);
    int input_constraint = kErrorStateDim + kHorizonSteps * kErrorStateDim;
    for (int step = 0; step < kHorizonSteps; ++step)
    {
      for (int index = 0; index < kInputDim; ++index, ++input_constraint)
      {
        lower_[input_constraint] = lower_input[step](index);
        upper_[input_constraint] = upper_input[step](index);
      }
    }
  }

  void setDesiredStart(const Eigen::Matrix<double, kStateDim, 1> &state,
                       const Eigen::Matrix<double, kInputDim, 1> &input)
  {
    desired_state_start_ = state;
    desired_input_start_ = input;
    desired_start_initialized_ = true;
  }

  bool getDesiredStart(Eigen::Matrix<double, kStateDim, 1> &state,
                       Eigen::Matrix<double, kInputDim, 1> &input) const
  {
    if (!desired_start_initialized_)
    {
      return false;
    }
    state = desired_state_start_;
    input = desired_input_start_;
    return true;
  }

  bool setInitialError(const Eigen::Matrix<double, kErrorStateDim, 1> &error)
  {
    if (lower_.size() != static_cast<std::size_t>(total_constraints_) || !error.allFinite())
    {
      return false;
    }
    for (int index = 0; index < kErrorStateDim; ++index)
    {
      lower_[index] = error(index);
      upper_[index] = error(index);
    }
    return true;
  }

  bool solve(MPCSolution &solution, std::string &error_message)
  {
    if (p_data_.empty() || a_data_.empty() ||
        q_.size() != static_cast<std::size_t>(total_variables_))
    {
      error_message = "OM-MPC matrices are not initialized";
      return false;
    }

    OSQPData data{};
    data.n = total_variables_;
    data.m = total_constraints_;
    data.P = csc_matrix(data.n, data.n, static_cast<c_int>(p_data_.size()),
                        p_data_.data(), p_indices_.data(), p_indptr_.data());
    data.q = q_.data();
    data.A = csc_matrix(data.m, data.n, static_cast<c_int>(a_data_.size()),
                        a_data_.data(), a_indices_.data(), a_indptr_.data());
    data.l = lower_.data();
    data.u = upper_.data();

    if (data.P == nullptr || data.A == nullptr)
    {
      if (data.P != nullptr)
        c_free(data.P);
      if (data.A != nullptr)
        c_free(data.A);
      error_message = "Failed to allocate OSQP CSC matrix wrappers";
      return false;
    }

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.polish = true;
    settings.verbose = false;

    OSQPWorkspace *workspace = nullptr;
    const c_int setup_result = osqp_setup(&workspace, &data, &settings);
    if (setup_result != 0 || workspace == nullptr)
    {
      c_free(data.P);
      c_free(data.A);
      std::ostringstream stream;
      stream << "OSQP setup failed with code " << setup_result;
      error_message = stream.str();
      return false;
    }

    const c_int solve_result = osqp_solve(workspace);
    bool success = solve_result == 0 && workspace->info != nullptr &&
                   (workspace->info->status_val == OSQP_SOLVED ||
                    workspace->info->status_val == OSQP_SOLVED_INACCURATE) &&
                   workspace->solution != nullptr && workspace->solution->x != nullptr;

    if (success)
    {
      extractSolution(workspace->solution->x, solution);
      solution.optimal_cost = workspace->info->obj_val;
      for (const auto &input : solution.delta_u)
      {
        if (!finiteVector(input))
        {
          success = false;
          error_message = "OSQP returned a non-finite control increment";
          break;
        }
      }
    }
    else
    {
      std::ostringstream stream;
      stream << "OSQP solve failed";
      if (workspace->info != nullptr && workspace->info->status != nullptr)
      {
        stream << ": " << workspace->info->status;
      }
      stream << " (code " << solve_result << ")";
      error_message = stream.str();
    }

    osqp_cleanup(workspace);
    // csc_matrix() allocates only these wrappers; their x/i/p arrays are owned
    // by the std::vectors above and must not be freed by csc_spfree().
    c_free(data.P);
    c_free(data.A);
    return success;
  }

private:
  void extractSolution(const c_float *raw_solution, MPCSolution &solution) const
  {
    solution.delta_x.resize(kHorizonSteps + 1);
    solution.delta_u.resize(kHorizonSteps);
    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const int state_offset = step * (kErrorStateDim + kInputDim);
      const int input_offset = state_offset + kErrorStateDim;
      solution.delta_x[step] = Eigen::Map<const Eigen::Matrix<c_float, kErrorStateDim, 1>>(
          raw_solution + state_offset).template cast<double>();
      solution.delta_u[step] = Eigen::Map<const Eigen::Matrix<c_float, kInputDim, 1>>(
          raw_solution + input_offset).template cast<double>();
    }
    const int terminal_offset = kHorizonSteps * (kErrorStateDim + kInputDim);
    solution.delta_x[kHorizonSteps] =
        Eigen::Map<const Eigen::Matrix<c_float, kErrorStateDim, 1>>(
            raw_solution + terminal_offset).template cast<double>();
  }

  int total_variables_;
  int total_constraints_;

  std::vector<c_float> p_data_;
  std::vector<c_int> p_indices_;
  std::vector<c_int> p_indptr_;
  std::vector<c_float> a_data_;
  std::vector<c_int> a_indices_;
  std::vector<c_int> a_indptr_;
  std::vector<c_float> q_;
  std::vector<c_float> lower_;
  std::vector<c_float> upper_;

  Eigen::Matrix<double, kStateDim, 1> desired_state_start_;
  Eigen::Matrix<double, kInputDim, 1> desired_input_start_;
  bool desired_start_initialized_{false};
};

} // namespace

class OnManifoldMPC::Impl
{
public:
  bool initialize(const Parameter_t &parameter)
  {
    gravity_ = parameter.gra;
    config_ = parameter.ommpc;

    if (!finiteScalar(gravity_) || gravity_ <= 0.0 ||
        !finiteScalar(config_.step_T) || config_.step_T <= 0.0 ||
        !finiteScalar(config_.solve_frequency) || config_.solve_frequency <= 0.0)
    {
      setError("Invalid gravity or OM-MPC timing parameter");
      return false;
    }
    if (!finiteScalar(config_.state_cost_exponential) ||
        !finiteScalar(config_.input_cost_exponential) ||
        config_.state_cost_exponential < 0.0 ||
        config_.input_cost_exponential < 0.0 ||
        !finiteScalar(config_.max_bodyrate_xy) || config_.max_bodyrate_xy <= 0.0 ||
        !finiteScalar(config_.max_bodyrate_z) || config_.max_bodyrate_z <= 0.0 ||
        !finiteScalar(config_.min_thrust) || config_.min_thrust <= 0.0 ||
        !finiteScalar(config_.max_thrust) || config_.max_thrust <= config_.min_thrust ||
        (config_.use_heartbeat &&
         (!finiteScalar(config_.heartbeat_timeout) || config_.heartbeat_timeout <= 0.0)))
    {
      setError("Invalid OM-MPC decay, input-bound, or heartbeat parameter");
      return false;
    }

    Eigen::Matrix<double, kErrorStateDim, 1> state_weights;
    state_weights << config_.Q_pos_xy, config_.Q_pos_xy, config_.Q_pos_z,
        config_.Q_velocity, config_.Q_velocity, config_.Q_velocity,
        config_.Q_attitude_rp, config_.Q_attitude_rp, config_.Q_attitude_yaw;
    Eigen::Matrix<double, kInputDim, 1> input_weights;
    input_weights << config_.R_thrust, config_.R_pitchroll,
        config_.R_pitchroll, config_.R_yaw;
    if (!state_weights.allFinite() || !input_weights.allFinite() ||
        (state_weights.array() <= 0.0).any() || (input_weights.array() <= 0.0).any())
    {
      setError("OM-MPC cost weights must be finite and positive");
      return false;
    }

    wrapper_.buildHessian(state_weights, input_weights,
                          config_.state_cost_exponential,
                          config_.input_cost_exponential);
    state_matrices_.resize(kHorizonSteps);
    input_matrices_.resize(kHorizonSteps);
    lower_input_.resize(kHorizonSteps);
    upper_input_.resize(kHorizonSteps);
    initialized_ = true;
    clearError();
    return true;
  }

  void feedTrajectory(const traj_utils::PolyTrajConstPtr &message)
  {
    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "[px4ctrl][OM-MPC] Controller is not initialized.");
      return;
    }
    if (!message)
    {
      setError("received a null PolyTraj pointer");
      ROS_ERROR("[px4ctrl][OM-MPC] Rejected null PolyTraj pointer.");
      return;
    }

    PolynomialTrajectory parsed_trajectory;
    std::string validation_error;
    if (!parseTrajectory(*message, parsed_trajectory, validation_error))
    {
      setError(validation_error);
      ROS_ERROR_STREAM("[px4ctrl][OM-MPC] Rejected PolyTraj: " << validation_error);
      return;
    }

    ScheduledTrajectory incoming;
    incoming.trajectory = std::move(parsed_trajectory);
    incoming.start_time = message->start_time;
    incoming.end_time = incoming.start_time +
                        ros::Duration(incoming.trajectory.totalDuration());
    incoming.id = message->traj_id;

    const ros::Time receipt_time = ros::Time::now();
    const ScheduledTrajectory *latest_started = nullptr;
    for (const auto &scheduled : trajectories_)
    {
      if (scheduled.start_time <= receipt_time)
        latest_started = &scheduled;
      else
        break;
    }
    if (latest_started != nullptr && incoming.start_time < latest_started->start_time)
    {
      ROS_WARN_STREAM("[px4ctrl][OM-MPC] Ignoring stale PolyTraj id="
                      << incoming.id << " with an older start time.");
      return;
    }

    for (const auto &scheduled : trajectories_)
    {
      if (scheduled.start_time == incoming.start_time && scheduled.id >= incoming.id)
      {
        ROS_WARN_STREAM("[px4ctrl][OM-MPC] Ignoring duplicate/out-of-order PolyTraj id="
                        << incoming.id);
        return;
      }
    }

    if (trajectory_invalidated_)
      trajectories_.clear();

    if (incoming.start_time > receipt_time)
    {
      // A newly received future segment supersedes any future plan at or after
      // its start, but the currently executing segment remains active.
      trajectories_.erase(
          std::remove_if(trajectories_.begin(), trajectories_.end(),
                         [&](const ScheduledTrajectory &scheduled) {
                           return scheduled.start_time >= incoming.start_time;
                         }),
          trajectories_.end());
    }
    else
    {
      // A current replan supersedes all older/current segments while preserving
      // a separately scheduled future segment.
      trajectories_.erase(
          std::remove_if(trajectories_.begin(), trajectories_.end(),
                         [&](const ScheduledTrajectory &scheduled) {
                           return scheduled.start_time <= incoming.start_time;
                         }),
          trajectories_.end());
    }

    const auto insertion_point = std::lower_bound(
        trajectories_.begin(), trajectories_.end(), incoming.start_time,
        [](const ScheduledTrajectory &scheduled, const ros::Time &start_time) {
          return scheduled.start_time < start_time;
        });
    const int accepted_id = incoming.id;
    const double accepted_duration = incoming.trajectory.totalDuration();
    trajectories_.insert(insertion_point, std::move(incoming));
    trajectory_invalidated_ = false;
    cached_output_valid_ = false;
    clearError();

    ROS_INFO_STREAM("[px4ctrl][OM-MPC] Accepted PolyTraj id=" << accepted_id
                    << ", pieces=" << message->duration.size()
                    << ", duration=" << accepted_duration
                    << " s, scheduled segments=" << trajectories_.size() << ".");
  }

  void feedHeartbeat()
  {
    last_heartbeat_time_ = ros::Time::now();
  }

  bool trajectoryReady(const ros::Time &now) const
  {
    return initialized_ && !trajectory_invalidated_ &&
           activeTrajectory(now) != nullptr && !heartbeatTimedOut(now);
  }

  bool trajectoryFinished(const ros::Time &now) const
  {
    return !trajectories_.empty() && !trajectory_invalidated_ &&
           now > trajectories_.back().end_time;
  }

  bool heartbeatTimedOut(const ros::Time &now) const
  {
    if (!config_.use_heartbeat)
    {
      return false;
    }
    return last_heartbeat_time_.isZero() ||
           (now - last_heartbeat_time_).toSec() > config_.heartbeat_timeout;
  }

  bool getTrajectoryEndPosition(Eigen::Vector3d &position) const
  {
    if (trajectories_.empty() || trajectories_.back().trajectory.empty())
    {
      return false;
    }
    position = trajectories_.back().trajectory.endPosition();
    return position.allFinite();
  }

  void invalidateTrajectory(const std::string &reason)
  {
    trajectory_invalidated_ = true;
    trajectories_.clear();
    cached_output_valid_ = false;
    setError(reason);
    ROS_ERROR_STREAM("[px4ctrl][OM-MPC] Trajectory invalidated: " << reason);
  }

  bool update(const Eigen::Vector3d &position,
              const Eigen::Vector3d &velocity,
              const Eigen::Quaterniond &attitude,
              const ros::Time &now,
              ControlOutput &output)
  {
    const DebugOutput previous_debug = last_debug_;
    last_debug_ = DebugOutput();
    const ScheduledTrajectory *active_trajectory = activeTrajectory(now);
    if (active_trajectory != nullptr)
    {
      last_debug_.trajectory_id = active_trajectory->id;
      last_debug_.trajectory_time = (now - active_trajectory->start_time).toSec();
    }
    if (!initialized_ || trajectory_invalidated_ || active_trajectory == nullptr ||
        heartbeatTimedOut(now))
    {
      setError(heartbeatTimedOut(now) ? "planner heartbeat timed out" :
                                       "no active PolyTraj at the current time");
      return false;
    }
    if (!position.allFinite() || !velocity.allFinite() || !attitude.coeffs().allFinite() ||
        attitude.norm() < 1.0e-6)
    {
      setError("non-finite odometry supplied to OM-MPC");
      return false;
    }

    if (reference_trajectory_start_time_ != active_trajectory->start_time ||
        reference_trajectory_id_ != active_trajectory->id)
    {
      reference_trajectory_start_time_ = active_trajectory->start_time;
      reference_trajectory_id_ = active_trajectory->id;
      cached_output_valid_ = false;
      yaw_initialized_ = false;
    }

    const double solve_period = 1.0 / config_.solve_frequency;
    if (cached_output_valid_ && !last_solve_time_.isZero())
    {
      const double elapsed = (now - last_solve_time_).toSec();
      if (elapsed >= 0.0 && elapsed < solve_period)
      {
        output = cached_output_;
        last_debug_ = previous_debug;
        last_debug_.trajectory_id = active_trajectory->id;
        last_debug_.trajectory_time = (now - active_trajectory->start_time).toSec();
        last_debug_.solve_time_ms = 0.0;
        last_debug_.commanded_collective_acceleration = output.collective_acceleration;
        last_debug_.commanded_bodyrates = output.bodyrates;
        return true;
      }
    }

    const auto solve_start = std::chrono::steady_clock::now();
    Eigen::Quaterniond normalized_attitude = attitude.normalized();
    const double trajectory_time = (now - active_trajectory->start_time).toSec();
    if (!setTrajectoryReference(active_trajectory->trajectory, trajectory_time,
                                normalized_attitude))
    {
      return false;
    }

    Eigen::Matrix<double, kStateDim, 1> desired_state;
    Eigen::Matrix<double, kInputDim, 1> desired_input;
    if (!wrapper_.getDesiredStart(desired_state, desired_input))
    {
      setError("OM-MPC desired state was not initialized");
      return false;
    }

    Eigen::Quaterniond desired_attitude(desired_state(3), desired_state(4),
                                        desired_state(5), desired_state(6));
    desired_attitude.normalize();
    const Eigen::Vector3d attitude_error = so3Log(
        normalized_attitude.toRotationMatrix().transpose() *
        desired_attitude.toRotationMatrix());

    Eigen::Matrix<double, kErrorStateDim, 1> initial_error;
    initial_error << desired_state.template head<3>() - position,
        desired_state.template tail<3>() - velocity,
        attitude_error;
    last_debug_.reference_valid = true;
    last_debug_.reference_position = desired_state.template head<3>();
    last_debug_.reference_velocity = desired_state.template tail<3>();
    last_debug_.reference_attitude = desired_attitude;
    last_debug_.reference_collective_acceleration = desired_input(0);
    last_debug_.reference_bodyrates = desired_input.template tail<3>();
    last_debug_.reference_acceleration =
        desired_attitude * Eigen::Vector3d::UnitZ() * desired_input(0) -
        gravity_ * Eigen::Vector3d::UnitZ();
    last_debug_.position_error = initial_error.template head<3>();
    last_debug_.velocity_error = initial_error.template segment<3>(3);
    last_debug_.attitude_error = attitude_error;
    if (!initial_error.allFinite() || !wrapper_.setInitialError(initial_error))
    {
      setError("failed to construct a finite OM-MPC initial error");
      return false;
    }

    MPCSolution solution;
    std::string solver_error;
    const auto qp_solve_start = std::chrono::steady_clock::now();
    const bool qp_solved = wrapper_.solve(solution, solver_error);
    const auto qp_solve_end = std::chrono::steady_clock::now();
    last_debug_.solve_time_ms =
        std::chrono::duration<double, std::milli>(qp_solve_end - qp_solve_start).count();
    if (!qp_solved || solution.delta_u.empty())
    {
      setError(solver_error);
      return false;
    }
    last_debug_.solver_success = true;
    last_debug_.optimal_cost = solution.optimal_cost;

    output.collective_acceleration = desired_input(0) - solution.delta_u.front()(0);
    output.bodyrates = desired_input.template tail<3>() -
                       solution.delta_u.front().template tail<3>();

    output.collective_acceleration =
        std::max(config_.min_thrust,
                 std::min(config_.max_thrust, output.collective_acceleration));
    output.bodyrates.x() = std::max(-config_.max_bodyrate_xy,
                                    std::min(config_.max_bodyrate_xy, output.bodyrates.x()));
    output.bodyrates.y() = std::max(-config_.max_bodyrate_xy,
                                    std::min(config_.max_bodyrate_xy, output.bodyrates.y()));
    output.bodyrates.z() = std::max(-config_.max_bodyrate_z,
                                    std::min(config_.max_bodyrate_z, output.bodyrates.z()));
    last_debug_.commanded_collective_acceleration = output.collective_acceleration;
    last_debug_.commanded_bodyrates = output.bodyrates;
    if (!finiteScalar(output.collective_acceleration) || !output.bodyrates.allFinite())
    {
      setError("OM-MPC produced a non-finite control command");
      return false;
    }

    cached_output_ = output;
    cached_output_valid_ = true;
    last_solve_time_ = now;
    const auto solve_end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
    timing_ms_ = 0.9 * timing_ms_ + 0.1 * elapsed_ms;
    clearError();
    return true;
  }

  const std::string &lastError() const
  {
    return last_error_;
  }

  double timingMilliseconds() const
  {
    return timing_ms_;
  }

  const DebugOutput &debugOutput() const
  {
    return last_debug_;
  }

private:
  const ScheduledTrajectory *activeTrajectory(const ros::Time &now) const
  {
    const ScheduledTrajectory *active = nullptr;
    std::size_t active_index = 0;
    for (std::size_t index = 0; index < trajectories_.size(); ++index)
    {
      if (trajectories_[index].start_time <= now)
      {
        active = &trajectories_[index];
        active_index = index;
      }
      else
      {
        break;
      }
    }
    if (active == nullptr)
      return nullptr;

    // Match the original OM-MPC scheduler: when a future replan is queued,
    // clamp/hold the current segment at its endpoint until the next start time.
    const ros::Time effective_end = active_index + 1 < trajectories_.size()
                                        ? trajectories_[active_index + 1].start_time
                                        : active->end_time;
    return now <= effective_end ? active : nullptr;
  }

  bool parseTrajectory(const traj_utils::PolyTraj &message,
                       PolynomialTrajectory &trajectory,
                       std::string &error) const
  {
    if (message.traj_id < 1)
    {
      error = "trajectory id must be positive";
      return false;
    }
    if (message.start_time.isZero())
    {
      error = "trajectory start_time is zero";
      return false;
    }
    if (message.order < 3 || message.order > kMaxPolynomialOrder)
    {
      std::ostringstream stream;
      stream << "unsupported polynomial order " << static_cast<int>(message.order)
             << " (expected 3.." << kMaxPolynomialOrder << ")";
      error = stream.str();
      return false;
    }
    if (message.duration.empty())
    {
      error = "trajectory contains no polynomial pieces";
      return false;
    }

    const std::size_t coefficients_per_piece = static_cast<std::size_t>(message.order) + 1;
    if (message.duration.size() >
        std::numeric_limits<std::size_t>::max() / coefficients_per_piece)
    {
      error = "trajectory coefficient count overflows size_t";
      return false;
    }
    const std::size_t expected_coefficients = message.duration.size() * coefficients_per_piece;
    if (message.coef_x.size() != expected_coefficients ||
        message.coef_y.size() != expected_coefficients ||
        message.coef_z.size() != expected_coefficients)
    {
      std::ostringstream stream;
      stream << "coefficient length mismatch: expected " << expected_coefficients
             << " coefficients per axis";
      error = stream.str();
      return false;
    }

    trajectory.clear();
    double total_duration = 0.0;
    for (std::size_t piece = 0; piece < message.duration.size(); ++piece)
    {
      const double duration = message.duration[piece];
      if (!finiteScalar(duration) || duration < kMinTrajectoryDuration)
      {
        error = "piece duration must be finite and positive";
        return false;
      }
      total_duration += duration;
      if (!finiteScalar(total_duration))
      {
        error = "total trajectory duration is not finite";
        return false;
      }

      Eigen::MatrixXd coefficients(3, coefficients_per_piece);
      for (std::size_t coefficient = 0; coefficient < coefficients_per_piece; ++coefficient)
      {
        const std::size_t index = piece * coefficients_per_piece + coefficient;
        const double x = message.coef_x[index];
        const double y = message.coef_y[index];
        const double z = message.coef_z[index];
        if (!finiteScalar(x) || !finiteScalar(y) || !finiteScalar(z))
        {
          error = "trajectory contains a NaN or infinite coefficient";
          return false;
        }
        coefficients(0, coefficient) = x;
        coefficients(1, coefficient) = y;
        coefficients(2, coefficient) = z;
      }
      trajectory.addPiece(duration, coefficients);
    }
    return !trajectory.empty();
  }

  bool setTrajectoryReference(const PolynomialTrajectory &trajectory,
                              const double start_time,
                              const Eigen::Quaterniond &estimated_attitude)
  {
    double yaw_cursor = yaw_initialized_ ? reference_yaw_ :
                                           yawFromQuaternion(estimated_attitude);
    double yaw_rate_cursor = yaw_initialized_ ? reference_yaw_rate_ : 0.0;
    double first_yaw = yaw_cursor;
    double first_yaw_rate = yaw_rate_cursor;

    for (int step = 0; step < kHorizonSteps; ++step)
    {
      const double sample_time = start_time + step * config_.step_T;
      const bool at_end = sample_time >= trajectory.totalDuration();
      const double clamped_time = std::max(0.0, std::min(trajectory.totalDuration(), sample_time));
      const Eigen::Vector3d desired_position = trajectory.evaluate(clamped_time, 0);
      const Eigen::Vector3d desired_velocity =
          at_end ? Eigen::Vector3d::Zero() : trajectory.evaluate(clamped_time, 1);
      const Eigen::Vector3d desired_acceleration =
          at_end ? Eigen::Vector3d::Zero() : trajectory.evaluate(clamped_time, 2);
      const Eigen::Vector3d desired_jerk =
          at_end ? Eigen::Vector3d::Zero() : trajectory.evaluate(clamped_time, 3);

      if (!desired_position.allFinite() || !desired_velocity.allFinite() ||
          !desired_acceleration.allFinite() || !desired_jerk.allFinite())
      {
        setError("non-finite value while evaluating PolyTraj");
        return false;
      }

      double yaw = 0.0;
      double yaw_rate = 0.0;
      if (!config_.use_fix_yaw)
      {
        advanceYaw(desired_velocity, config_.step_T,
                   yaw_cursor, yaw_rate_cursor, yaw, yaw_rate);
      }
      if (step == 0)
      {
        first_yaw = yaw;
        first_yaw_rate = yaw_rate;
      }

      const Eigen::Vector3d total_acceleration =
          desired_acceleration + gravity_ * Eigen::Vector3d::UnitZ();
      // For a flatness-consistent nominal state, R_d e3 is parallel to the
      // total acceleration, hence the mass-normalized collective thrust is
      // its norm. Projecting onto the previous body z-axis is incorrect.
      const double collective_acceleration = total_acceleration.norm();
      if (!finiteScalar(collective_acceleration) ||
          collective_acceleration < kMinCollectiveAcceleration)
      {
        setError("PolyTraj requires near-zero collective acceleration");
        return false;
      }

      Eigen::Quaterniond desired_attitude;
      Eigen::Vector3d desired_bodyrates;
      if (!flatnessReference(total_acceleration, desired_jerk, yaw, yaw_rate,
                             desired_attitude, desired_bodyrates))
      {
        setError("failed to construct a nonsingular flatness reference");
        return false;
      }

      setStateMatricesAndBounds(step, desired_attitude, desired_bodyrates,
                                collective_acceleration);
      if (step == 0)
      {
        Eigen::Matrix<double, kStateDim, 1> desired_state;
        desired_state << desired_position,
            desired_attitude.w(), desired_attitude.x(),
            desired_attitude.y(), desired_attitude.z(),
            desired_velocity;
        Eigen::Matrix<double, kInputDim, 1> desired_input;
        desired_input << collective_acceleration, desired_bodyrates;
        wrapper_.setDesiredStart(desired_state, desired_input);
      }
    }

    wrapper_.buildConstraints(state_matrices_, input_matrices_,
                              lower_input_, upper_input_);
    if (!config_.use_fix_yaw)
    {
      reference_yaw_ = first_yaw;
      reference_yaw_rate_ = first_yaw_rate;
      yaw_initialized_ = true;
    }
    return true;
  }

  bool flatnessReference(const Eigen::Vector3d &total_acceleration,
                         const Eigen::Vector3d &jerk,
                         const double yaw,
                         const double yaw_rate,
                         Eigen::Quaterniond &attitude,
                         Eigen::Vector3d &bodyrates) const
  {
    const double acceleration_norm = total_acceleration.norm();
    if (!finiteScalar(acceleration_norm) || acceleration_norm < kMinCollectiveAcceleration)
    {
      return false;
    }

    const Eigen::Vector3d body_z = total_acceleration / acceleration_norm;
    if (1.0 + body_z.z() < kHopfSingularityTolerance)
    {
      // The chosen Hopf chart is singular at exact inverted thrust. A diff_planner
      // trajectory is not expected to contain inverted flight; fail safely instead
      // of reusing a stale nominal attitude.
      return false;
    }

    const Eigen::Vector3d body_z_rate =
        (Eigen::Matrix3d::Identity() - body_z * body_z.transpose()) *
        jerk / acceleration_norm;
    const double normalization = std::sqrt(2.0 * (1.0 + body_z.z()));
    const Eigen::Quaterniond tilt((1.0 + body_z.z()) / normalization,
                                  -body_z.y() / normalization,
                                  body_z.x() / normalization,
                                  0.0);
    const Eigen::Quaterniond yaw_quaternion(std::cos(0.5 * yaw), 0.0, 0.0,
                                            std::sin(0.5 * yaw));
    attitude = (tilt * yaw_quaternion).normalized();

    const double sine_yaw = std::sin(yaw);
    const double cosine_yaw = std::cos(yaw);
    const double denominator = 1.0 + body_z.z();
    bodyrates.x() = sine_yaw * body_z_rate.x() - cosine_yaw * body_z_rate.y() -
                    (body_z.x() * sine_yaw - body_z.y() * cosine_yaw) *
                        body_z_rate.z() / denominator;
    bodyrates.y() = cosine_yaw * body_z_rate.x() + sine_yaw * body_z_rate.y() -
                    (body_z.x() * cosine_yaw + body_z.y() * sine_yaw) *
                        body_z_rate.z() / denominator;
    bodyrates.z() = (body_z.y() * body_z_rate.x() -
                     body_z.x() * body_z_rate.y()) /
                        denominator +
                    yaw_rate;
    return attitude.coeffs().allFinite() && bodyrates.allFinite();
  }

  void setStateMatricesAndBounds(const int step,
                                 const Eigen::Quaterniond &attitude,
                                 const Eigen::Vector3d &bodyrates,
                                 const double collective_acceleration)
  {
    Eigen::SparseMatrix<double> &state_matrix = state_matrices_[step];
    Eigen::SparseMatrix<double> &input_matrix = input_matrices_[step];
    state_matrix.resize(kErrorStateDim, kErrorStateDim);
    input_matrix.resize(kErrorStateDim, kInputDim);

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(30);
    for (int axis = 0; axis < 3; ++axis)
    {
      entries.emplace_back(axis, axis, 1.0);
      entries.emplace_back(3 + axis, 3 + axis, 1.0);
      entries.emplace_back(axis, 3 + axis, config_.step_T);
    }

    const Eigen::Matrix3d attitude_transition = so3Exp(-bodyrates * config_.step_T);
    const Eigen::Matrix3d velocity_attitude =
        config_.step_T * attitude.toRotationMatrix() *
        hat(Eigen::Vector3d(0.0, 0.0, -collective_acceleration));
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        entries.emplace_back(6 + row, 6 + column,
                             attitude_transition(row, column));
        entries.emplace_back(3 + row, 6 + column,
                             velocity_attitude(row, column));
      }
    }
    state_matrix.setFromTriplets(entries.begin(), entries.end());
    state_matrix.makeCompressed();

    entries.clear();
    entries.reserve(12);
    const Eigen::Vector3d thrust_input =
        config_.step_T * attitude.toRotationMatrix() * Eigen::Vector3d::UnitZ();
    for (int row = 0; row < 3; ++row)
    {
      entries.emplace_back(3 + row, 0, thrust_input(row));
    }
    const Eigen::Matrix3d rate_input =
        so3LeftJacobian(bodyrates * config_.step_T).transpose() * config_.step_T;
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        entries.emplace_back(6 + row, 1 + column, rate_input(row, column));
      }
    }
    input_matrix.setFromTriplets(entries.begin(), entries.end());
    input_matrix.makeCompressed();

    // delta_u = u_desired - u_actual
    lower_input_[step] << collective_acceleration - config_.max_thrust,
        bodyrates.x() - config_.max_bodyrate_xy,
        bodyrates.y() - config_.max_bodyrate_xy,
        bodyrates.z() - config_.max_bodyrate_z;
    upper_input_[step] << collective_acceleration - config_.min_thrust,
        bodyrates.x() + config_.max_bodyrate_xy,
        bodyrates.y() + config_.max_bodyrate_xy,
        bodyrates.z() + config_.max_bodyrate_z;
  }

  void advanceYaw(const Eigen::Vector3d &velocity,
                  const double time_step,
                  double &yaw_cursor,
                  double &yaw_rate_cursor,
                  double &yaw,
                  double &yaw_rate) const
  {
    const double target_yaw = velocity.head<2>().norm() > 0.1
                                  ? std::atan2(velocity.y(), velocity.x())
                                  : yaw_cursor;
    double yaw_difference = normalizeAngle(target_yaw - yaw_cursor);
    const double max_rate = 0.9 * config_.max_bodyrate_z;
    const double max_acceleration = 4.0 * config_.max_bodyrate_z;
    const double signed_acceleration = yaw_difference >= 0.0
                                           ? max_acceleration
                                           : -max_acceleration;
    const double signed_max_rate = yaw_difference >= 0.0 ? max_rate : -max_rate;

    double allowed_difference;
    const double time_to_rate_limit =
        (signed_max_rate - yaw_rate_cursor) / signed_acceleration;
    if (time_to_rate_limit >= 0.0 && time_to_rate_limit < time_step)
    {
      allowed_difference = yaw_rate_cursor * time_to_rate_limit +
                           0.5 * signed_acceleration * time_to_rate_limit *
                               time_to_rate_limit +
                           signed_max_rate * (time_step - time_to_rate_limit);
    }
    else
    {
      allowed_difference = yaw_rate_cursor * time_step +
                           0.5 * signed_acceleration * time_step * time_step;
    }
    if (std::fabs(yaw_difference) > std::fabs(allowed_difference))
    {
      yaw_difference = allowed_difference;
    }

    yaw_rate = yaw_difference / time_step;
    yaw = normalizeAngle(yaw_cursor + yaw_difference);
    yaw_cursor = yaw;
    yaw_rate_cursor = yaw_rate;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  static double yawFromQuaternion(const Eigen::Quaterniond &attitude)
  {
    return std::atan2(2.0 * (attitude.w() * attitude.z() +
                            attitude.x() * attitude.y()),
                      1.0 - 2.0 * (attitude.y() * attitude.y() +
                                   attitude.z() * attitude.z()));
  }

  void setError(const std::string &error)
  {
    last_error_ = error;
  }

  void clearError()
  {
    last_error_.clear();
  }

  Parameter_t::OnManifoldMPC config_{};
  double gravity_{9.81};
  bool initialized_{false};

  std::vector<ScheduledTrajectory> trajectories_;
  bool trajectory_invalidated_{false};
  ros::Time last_heartbeat_time_{0};

  MPCWrapper wrapper_;
  std::vector<Eigen::SparseMatrix<double>> state_matrices_;
  std::vector<Eigen::SparseMatrix<double>> input_matrices_;
  std::vector<Eigen::Matrix<double, kInputDim, 1>> lower_input_;
  std::vector<Eigen::Matrix<double, kInputDim, 1>> upper_input_;

  ControlOutput cached_output_;
  bool cached_output_valid_{false};
  ros::Time last_solve_time_{0};
  ros::Time reference_trajectory_start_time_{0};
  int reference_trajectory_id_{0};
  double timing_ms_{0.0};

  bool yaw_initialized_{false};
  double reference_yaw_{0.0};
  double reference_yaw_rate_{0.0};
  DebugOutput last_debug_;
  std::string last_error_;
};

OnManifoldMPC::OnManifoldMPC() : impl_(new Impl)
{
}

OnManifoldMPC::~OnManifoldMPC() = default;

bool OnManifoldMPC::initialize(const Parameter_t &param)
{
  return impl_->initialize(param);
}

void OnManifoldMPC::feedTrajectory(const traj_utils::PolyTrajConstPtr &msg)
{
  impl_->feedTrajectory(msg);
}

void OnManifoldMPC::feedHeartbeat(const std_msgs::EmptyConstPtr &msg)
{
  (void)msg;
  impl_->feedHeartbeat();
}

bool OnManifoldMPC::trajectoryReady(const ros::Time &now) const
{
  return impl_->trajectoryReady(now);
}

bool OnManifoldMPC::trajectoryFinished(const ros::Time &now) const
{
  return impl_->trajectoryFinished(now);
}

bool OnManifoldMPC::heartbeatTimedOut(const ros::Time &now) const
{
  return impl_->heartbeatTimedOut(now);
}

bool OnManifoldMPC::getTrajectoryEndPosition(Eigen::Vector3d &position) const
{
  return impl_->getTrajectoryEndPosition(position);
}

void OnManifoldMPC::invalidateTrajectory(const std::string &reason)
{
  impl_->invalidateTrajectory(reason);
}

bool OnManifoldMPC::update(const Eigen::Vector3d &position,
                           const Eigen::Vector3d &velocity,
                           const Eigen::Quaterniond &attitude,
                           const ros::Time &now,
                           ControlOutput &output)
{
  return impl_->update(position, velocity, attitude, now, output);
}

const std::string &OnManifoldMPC::lastError() const
{
  return impl_->lastError();
}

double OnManifoldMPC::timingMilliseconds() const
{
  return impl_->timingMilliseconds();
}

const DebugOutput &OnManifoldMPC::debugOutput() const
{
  return impl_->debugOutput();
}

} // namespace px4ctrl_ommpc

# autonomy

ROS 1 workspace for autonomous UAV planning, LiDAR-inertial odometry, flight control, simulation, and command tools.

## Repository layout

```text
autonomy/
├── sh_files/                  # Local startup, recording, takeoff, and landing scripts
└── src/
    ├── diff_planner/          # Original upstream Diff-Planner packages
    ├── realflight_modules/
    │   ├── faster-lio/       # Local IMU-rate odometry propagation
    │   └── px4ctrl/          # Local controller with OMMPC support
    ├── uav_simulator/
    ├── user_command/          # Local periodic goal republishing
    └── Utils/
```

`src/diff_planner` is based on
[DifferentialRobotics/Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)
commit `5f8551203426b371c22de55e5961d04cf7639c60`. Local dynamic-obstacle packages and planner changes are intentionally excluded. Only the real-flight parameters in `run_exp_single_lio.launch` and `advanced_param_exp.xml` are carried over from the local aircraft configuration.

## LiDAR odometry wiring

Faster-LIO publishes the high-rate body odometry topic:

```text
/lidar_slam/imu_propagate
```

The LIO planner, px4ctrl, multipoint command node, and swarm bridge all consume this topic directly; the startup scripts no longer launch the separate `ekf` node for high-frequency odometry.

The propagation path is driven once per accepted IMU sample and is corrected from the latest complete Faster-LIO state after every LiDAR update. In `src/realflight_modules/faster-lio/faster-lio/config/mid360.yaml`:

- `publish/imu_prop_enable` enables propagation.
- `publish/imu_prop_topic` selects the odometry topic.
- `publish/imu_prop_use_filtered_imu` selects filtered or original IMU data for propagation only.
- `publish/imu_prop_filter_alpha` controls the EMA coefficient in `(0, 1]`; `1.0` disables smoothing strength while retaining the filtered path.

## Build

From the repository root in a configured ROS 1 environment:

```bash
catkin_make
source devel/setup.bash
```

For a single LIO flight stack:

```bash
bash sh_files/run_single_lio.sh
```

Scripts that access serial devices invoke `sudo` interactively. No sudo password is stored in this repository.

## Local extensions retained

- `realflight_modules/faster-lio`: IMU-rate, complete-state odometry propagation with optional propagation-only IMU filtering.
- `realflight_modules/px4ctrl`: local OMMPC controller additions.
- `Utils/quadrotor_msgs/msg/Px4ctrlDebug.msg`: OMMPC debug fields required by the local px4ctrl implementation.
- `user_command/multipoint`: periodically republishes the current goal using `goal_publish_interval`.

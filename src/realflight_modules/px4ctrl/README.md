<img src="images/nus_logo.png" alt="nus logo" align="right" height="80" />

# px4ctrl

## 概述
**px4ctrl** 是**微分智飞**公司旗下教育无人机子品牌**非凸空间**适配的飞机控制模块

## 运行环境
本项目基于ROS1开发，请根据所使用ubuntu版本安装对应版本ROS1，支持ubuntu16.04, 18.04和20.04。

## 项目拉取
- `px4ctrl` 模块依赖 `utils` 模块编译，所以需要将 `utils` 模块一并拉下

```bash
# 拉取 px4ctrl 模块
mkdir -p px4ctrl_ws/src
cd px4ctrl_ws/src
git clone https://github.com/DifferentialRobotics/px4ctrl.git  # 拉取 px4ctrl 模块
git clone https://github.com/DifferentialRobotics/utils.git  # 拉取 Utils 模块

cd ..
catkin_make
```

## 程序运行
- `px4ctrl` 模块依赖 `mavros, Faster-lio` 模块运行
    - `mavros` 模块默认安装于飞机`/opt/ros/noetic/share/mavros` 目录下
    - `Faster-lio` 链接：https://github.com/DifferentialRobotics/faster-lio.git

```bash
# 运行 px4ctrl 模块
cd px4ctrl_ws

source devel/setup.zsh  # 如果使用bash终端，则执行: source devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

`px4ctrl` 模块依靠 `ekf_pose` 输出的位置信息和 `Diff-Planner` 模块输出的目标点信息来输出对飞机的控制指令

## OM-MPC 

融合后的 OM-MPC 是 `pose_solver: 3`。它直接解析 Diff-Planner 发布的
`traj_utils/PolyTraj`，不依赖 `traj_server` 生成的 `PositionCommand`。

控制器分工如下：

| px4ctrl 状态 | `pose_solver: 3` 时实际控制器 | MAVROS 指令 |
| --- | --- | --- |
| `CMD_CTRL` 且 PolyTraj 当前有效、规划器心跳正常 | OM-MPC | 机体系角速度 + 推力 |
| `MANUAL_CTRL`、`AUTO_HOVER`、`AUTO_TAKEOFF`、`AUTO_LAND` | alg0（非 Offboard 时仍遵循原 FSM） | 姿态 + 推力 |
| 轨迹结束、心跳超时或 MPC 求解/参考构造失败 | 同周期切回 alg0 悬停 | 姿态 + 推力 |

OM-MPC 会校验轨迹编号、起始时间、阶数、分段时长、系数数量及有限性。
未来起始的重规划轨迹会排队，当前轨迹在切换时间之前继续执行；若当前段已到末端，
则保持其末端状态。延迟到达的旧轨迹不会覆盖当前轨迹。

### 依赖与编译

除原 px4ctrl 依赖外，需要：

- Diff-Planner 工作空间提供的 `traj_utils`（其 `PolyTraj.msg`）。
- OSQP 0.6.x C API，可被 CMake 找到为 `osqp/osqp.h` 和 `libosqp`。

原独立目录 `ommpc_controller` 只作为参考源码保留，并通过 `CATKIN_IGNORE`
排除在当前工作空间构建之外，以避免其自带的重复 `traj_utils` 消息包与
Diff-Planner 的 `traj_utils` 冲突。

### 运行

VIO：

```bash
roslaunch px4ctrl run_ctrl_ommpc_vio.launch
```

LIO：

```bash
roslaunch px4ctrl run_ctrl_ommpc_lio.launch
```

两个 launch 默认直接订阅：

- PolyTraj：`/drone_0_planning/trajectory`
- 规划器心跳：`/drone_0_traj_server/heartbeat`

心跳话题的默认名字沿用 Diff-Planner launch 中对规划器心跳发布端的 remap；
数据由 Diff-Planner 本身发布，不要求启动 `traj_server`。如果你的 Diff-Planner
launch 使用其他命名，可通过 `trajectory_topic`、`heartbeat_topic` 参数覆盖：

```bash
roslaunch px4ctrl run_ctrl_ommpc_lio.launch \
  trajectory_topic:=/drone_0_planning/trajectory \
  heartbeat_topic:=/your/planner/heartbeat
```

OM-MPC 参数位于 `config/ctrl_param_fpv.yaml` 的 `ommpc` 节。默认固定世界系
yaw 为 0；将 `use_fix_yaw` 设为 `false` 时，yaw 会受限地跟随轨迹水平速度方向。
如系统没有可用的规划器心跳，可以显式将 `use_heartbeat` 设为 `false`，但会失去
规划器进程失联时自动回退悬停的保护。

### 圆形/8 字 PolyTraj 测试轨迹

`publish_polytraj.py` 会先读取一帧里程计，以当前 `(x, y)` 为轨迹起点、当前
姿态 yaw 为“前方”，并保证整个轨迹位于起点的前向半平面。默认保持当前高度，
生成位置、速度、加速度段间连续的五次多项式，并直接发布
`traj_utils/PolyTraj`。它还会在完整轨迹期间模拟规划器心跳，所以可直接配合
上面的 OM-MPC launch 使用。

脚本本身不发布 yaw：`PolyTraj` 消息只包含 xyz 多项式，读取到的当前 odom yaw
仅用于确定轨迹在水平面中的“前方”。真正的期望 yaw 由 OM-MPC 决定：

- `ommpc/use_fix_yaw: true`（默认）：整个跟踪过程固定为世界系 yaw=0，并不是
  固定为开始发布时的 odom yaw。
- `ommpc/use_fix_yaw: false`：OM-MPC 根据轨迹水平速度方向生成 yaw，并对 yaw
  角速度和角加速度进行限制。

例如，LIO 下发布半径 1.2 m、速度 0.6 m/s 的圆。圆心位于当前位置正前方
1.2 m，轨迹从圆的最后端开始，`turn` 决定初始沿机体左侧还是右侧运动：

```bash
rosrun px4ctrl publish_polytraj.py \
  --shape circle --radius 1.2 --speed 0.6 --turn left \
  --odom-topic /lidar_slam/imu_propagate
```

发布前向 Gerono 8 字轨迹；`radius` 是前向半跨度，轨迹前向范围为
`0～2*radius`，交叉点位于当前位置正前方 `radius` 处；`eight-width` 是左右
方向半宽：

```bash
rosrun px4ctrl publish_polytraj.py \
  --shape eight --radius 1.5 --eight-width 0.8 --speed 0.7 \
  --odom-topic /lidar_slam/imu_propagate
```

脚本参数如下：

| 参数 | 默认值 | 适用范围/单位 | 说明 |
| --- | --- | --- | --- |
| `--shape` | `circle` | `circle` / `eight` | 选择圆形或 8 字轨迹。 |
| `--radius` | `1.0` | `>0`，m | 圆的半径；8 字的前向半跨度。两种轨迹的前向范围均为 `0～2*radius`。 |
| `--eight-width` | 与 `radius` 相同 | `>0`，m | 8 字左右方向的半宽；圆形轨迹不使用。 |
| `--speed` | `0.5` | `>0`，m/s | 期望跟踪速度；多项式节点处严格等于该值，段内近似恒速。 |
| `--laps` | `1` | 正整数 | 圆的圈数或 8 字完整循环次数。 |
| `--pieces-per-lap` | `32` | 正整数 | 每圈/循环的五次多项式段数；越大越接近解析曲线，但消息和求值开销也越大。 |
| `--turn` | `left` | `left` / `right` | 圆从起点先向机体左侧或右侧运动；8 字不使用。 |
| `--z` | 当前 odom z | m | 固定世界系飞行高度。未指定时采用收到的当前高度。 |
| `--odom-topic` | `/lidar_slam/imu_propagate` | ROS topic | 获取轨迹起点和前向 yaw 的 `nav_msgs/Odometry` 话题。VIO 常用 `/vins/imu_propagate`。 |
| `--trajectory-topic` | `/drone_0_planning/trajectory` | ROS topic | `traj_utils/PolyTraj` 发布话题，应与 px4ctrl launch 的 `trajectory_topic` 一致。 |
| `--heartbeat-topic` | `/drone_0_traj_server/heartbeat` | ROS topic | 模拟规划器心跳的话题，应与 px4ctrl launch 的 `heartbeat_topic` 一致。 |
| `--heartbeat-rate` | `20.0` | `>0`，Hz | 模拟规划器心跳的发布频率。 |
| `--no-heartbeat` | 关闭 | flag | 指定后不发送模拟心跳；仅在真实规划器已发心跳或 `use_heartbeat=false` 时使用。 |
| `--start-delay` | `0.5` | `>0`，s | 发布消息后到轨迹 `start_time` 的预留时间。 |
| `--odom-timeout` | `5.0` | `>0`，s | 等待第一帧 odom 的最长时间。 |
| `--drone-id` | `0` | 整数 | 写入 `PolyTraj.drone_id`。 |
| `--traj-id` | `1` | 正整数 | 写入 `PolyTraj.traj_id`；OM-MPC 要求从正数开始。 |

发布脚本只模拟规划器输入，不会自动切换 OFFBOARD、解锁或改变遥控器档位；实际
飞行应先用较小速度/尺度，并确认 px4ctrl 已具备进入 `CMD_CTRL` 的条件。

### 与 alg0 的数值对照

`test/ommpc_controller_test.cpp` 使用同一参考状态和同一里程计输入，直接对比
OM-MPC 与 alg0 的质量归一化总推力和机体系角速度。默认参数下的代表结果如下：

| 工况 | OM-MPC | alg0 | 结论 |
| --- | ---: | ---: | --- |
| 悬停总推力加速度 | 9.81000 | 9.81000 | 一致 |
| 垂向加速总推力加速度 | 11.01000 | 11.01000 | 一致 |
| 耦合加速度/jerk 总推力加速度 | 10.31136 | 10.31136 | 一致 |
| `+x` 位置误差的 pitch rate | 0.72859 | 4.00000 | 同方向，OM-MPC 更保守 |
| `+x` 速度误差的 pitch rate | 0.46537 | 1.42440 | 同方向，OM-MPC 更保守 |
| `+z` 位置误差的总推力加速度 | 12.11751 | 11.91000 | 相差约 1.7% |
| roll 姿态误差的 roll rate | 0.47551 | 1.59957 | 同方向，OM-MPC 更保守 |
| 理想模型悬停恢复 2 s 后位置误差 | 0.00950 m | 0.00772 m | 闭环结果接近 |
| 理想模型悬停恢复位置 RMSE | 0.08978 m | 0.08075 m | 相差约 11.2% |

这说明两者的平坦性前馈在数值上对得上；反馈部分由于 OM-MPC 在 0.2 s 预测域内
分配修正，而 alg0 使用高增益级联反馈，所以不应期待瞬时角速度逐值相等。当前
OM-MPC 默认权重的横向/姿态修正约为 alg0 的 18%～33%。若需要接近 alg0 的响应
速度，应在闭环仿真中联合调整 `Q_pos_*`、`Q_velocity`、`Q_attitude_*` 和 `R_*`，
而不是只按单个静态点放大输出。理想刚体模型的 2 s 悬停恢复测试中，两者都将
初始 0.212 m 三维位置误差收敛到 1 cm 内，说明保守的瞬时输出并未造成数量级不同
的闭环恢复表现；该测试不包含电机时延、气动扰动和估计噪声，不能替代 SITL/实机。

运行全部 px4ctrl 测试：

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_px4ctrl
catkin_test_results
```

### OM-MPC 调试数据

OM-MPC 运行时仍使用原 `/debugPx4ctrl` 话题和
`quadrotor_msgs/Px4ctrlDebug` 消息。`ommpc_status` 的取值为：

- `0`：`OMMPC_DISABLED`，`pose_solver` 不是 3，OM-MPC 未构造、未订阅、未解算。
- `1`：`OMMPC_STANDBY`，已选择 OM-MPC，但当前不在有效 PolyTraj 的 `CMD_CTRL`。
- `2`：`OMMPC_ACTIVE`，本周期输出来自 OM-MPC。
- `3`：`OMMPC_FALLBACK`，OM-MPC 求解或参考生成失败，本周期已使用 alg0 悬停输出。

主要字段包括 `ommpc_ref_*`、`ommpc_err_*`、参考/输出集体加速度、参考/输出
角速度、`ommpc_cmd_throttle_raw`、最终 `ommpc_cmd_throttle`、油门饱和标志、
QP 耗时和最优代价。`ommpc_solve_time_ms == 0` 且状态为 ACTIVE 表示控制频率
高于 MPC 求解频率，本周期复用了上一条有效 MPC 命令，并不表示求解失败。

由于 `Px4ctrlDebug.msg` 的 MD5 会随这些字段变化，更新代码后必须先重建
`quadrotor_msgs`，并重新编译所有订阅 `/debugPx4ctrl` 的节点。

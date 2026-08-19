rosbag record --tcpnodelay /back_trigger \
/tf \
/tf_static \
/rosout \
/diagnostics \
/laserMapping/cloud_registered \
/laserMapping/odometry \
/laserMapping/path \
/lidar_slam/imu_propagate \
/dynamic_points \
/states \
/static_map \
/mot_map_cloud \
/box_edge \
/object_pose \
/drone_0_diff_planner_node/grid_map/occupancy \
/drone_0_diff_planner_node/grid_map/occupancy_inflate \
/drone_0_diff_planner_node/planning/trajectory \
/drone_0_diff_planner_node/planning/data_display \
/drone_0_diff_planner_node/optimal_list \
/drone_0_diff_planner_node/optimal_list_array \
/drone_0_diff_planner_node/global_list \
/drone_0_diff_planner_node/failed_list \
/drone_0_diff_planner_node/goal_point \
/goal \
/move_base_simple/goal \
/setpoints_cmd

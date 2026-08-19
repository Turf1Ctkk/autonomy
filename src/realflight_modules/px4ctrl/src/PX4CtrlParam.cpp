#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	read_essential_param(nh, "thrust_model/noisy_imu", thr_map.noisy_imu);

	if (pose_solver == 3)
	{
		read_essential_param(nh, "ommpc/step_T", ommpc.step_T);
		read_essential_param(nh, "ommpc/solve_frequency", ommpc.solve_frequency);
		read_essential_param(nh, "ommpc/Q_pos_xy", ommpc.Q_pos_xy);
		read_essential_param(nh, "ommpc/Q_pos_z", ommpc.Q_pos_z);
		read_essential_param(nh, "ommpc/Q_velocity", ommpc.Q_velocity);
		read_essential_param(nh, "ommpc/Q_attitude_rp", ommpc.Q_attitude_rp);
		read_essential_param(nh, "ommpc/Q_attitude_yaw", ommpc.Q_attitude_yaw);
		read_essential_param(nh, "ommpc/R_thrust", ommpc.R_thrust);
		read_essential_param(nh, "ommpc/R_pitchroll", ommpc.R_pitchroll);
		read_essential_param(nh, "ommpc/R_yaw", ommpc.R_yaw);
		read_essential_param(nh, "ommpc/state_cost_exponential", ommpc.state_cost_exponential);
		read_essential_param(nh, "ommpc/input_cost_exponential", ommpc.input_cost_exponential);
		read_essential_param(nh, "ommpc/max_bodyrate_xy", ommpc.max_bodyrate_xy);
		read_essential_param(nh, "ommpc/max_bodyrate_z", ommpc.max_bodyrate_z);
		read_essential_param(nh, "ommpc/min_thrust", ommpc.min_thrust);
		read_essential_param(nh, "ommpc/max_thrust", ommpc.max_thrust);
		read_essential_param(nh, "ommpc/use_fix_yaw", ommpc.use_fix_yaw);
		read_essential_param(nh, "ommpc/use_trajectory_ending_pos", ommpc.use_trajectory_ending_pos);
		read_essential_param(nh, "ommpc/use_heartbeat", ommpc.use_heartbeat);
		read_essential_param(nh, "ommpc/heartbeat_timeout", ommpc.heartbeat_timeout);

		if (ommpc.step_T <= 0.0 || ommpc.solve_frequency <= 0.0 ||
			ommpc.min_thrust <= 0.0 || ommpc.max_thrust <= ommpc.min_thrust ||
			ommpc.max_bodyrate_xy <= 0.0 || ommpc.max_bodyrate_z <= 0.0 ||
			ommpc.heartbeat_timeout <= 0.0)
		{
			ROS_ERROR("Invalid OM-MPC timing or input-bound parameters.");
			ROS_BREAK();
		}
	}
	

	max_angle /= (180.0 / M_PI);

	if (pose_solver < 0 || pose_solver > 3)
	{
		ROS_ERROR("pose_solver must be one of 0, 1, 2, or 3 (OM-MPC).");
		ROS_BREAK();
	}

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };

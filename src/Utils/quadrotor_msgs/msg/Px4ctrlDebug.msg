Header header

# OMMPC status constants
uint8 OMMPC_DISABLED=0
uint8 OMMPC_STANDBY=1
uint8 OMMPC_ACTIVE=2
uint8 OMMPC_FALLBACK=3

# --- Legacy fields (PID / ADRC) ---
float64 des_v_x
float64 des_v_y
float64 des_v_z

float64 fb_a_x
float64 fb_a_y
float64 fb_a_z

float64 adrc_dist_a_x
float64 adrc_dist_a_y
float64 adrc_dist_a_z

float64 adrc_comp_a_x
float64 adrc_comp_a_y
float64 adrc_comp_a_z

float64 des_a_x
float64 des_a_y
float64 des_a_z

float64 des_q_x
float64 des_q_y
float64 des_q_z
float64 des_q_w

float64 des_thr
float64 hover_percentage
float64 thr_scale_compensate
float64 voltage

float64 err_axisang_x
float64 err_axisang_y
float64 err_axisang_z
float64 err_axisang_ang

float64 fb_rate_x
float64 fb_rate_y
float64 fb_rate_z

float64 exec_err_axisang_x
float64 exec_err_axisang_y
float64 exec_err_axisang_z
float64 exec_err_axisang_ang

float64 fb_axisang_x
float64 fb_axisang_y
float64 fb_axisang_z
float64 fb_axisang_ang

# --- OMMPC debug fields ---
uint8 ommpc_status
bool  ommpc_reference_valid
bool  ommpc_solver_success
bool  ommpc_throttle_saturated
int32 ommpc_traj_id
float64 ommpc_traj_time
float64 ommpc_solve_time_ms
float64 ommpc_optimal_cost

float64 ommpc_ref_p_x
float64 ommpc_ref_p_y
float64 ommpc_ref_p_z
float64 ommpc_ref_v_x
float64 ommpc_ref_v_y
float64 ommpc_ref_v_z
float64 ommpc_ref_a_x
float64 ommpc_ref_a_y
float64 ommpc_ref_a_z
float64 ommpc_ref_q_w
float64 ommpc_ref_q_x
float64 ommpc_ref_q_y
float64 ommpc_ref_q_z

float64 ommpc_err_p_x
float64 ommpc_err_p_y
float64 ommpc_err_p_z
float64 ommpc_err_v_x
float64 ommpc_err_v_y
float64 ommpc_err_v_z
float64 ommpc_err_att_x
float64 ommpc_err_att_y
float64 ommpc_err_att_z

float64 ommpc_ref_collective_acc
float64 ommpc_cmd_collective_acc
float64 ommpc_cmd_throttle_raw
float64 ommpc_cmd_throttle

float64 ommpc_ref_rate_x
float64 ommpc_ref_rate_y
float64 ommpc_ref_rate_z
float64 ommpc_cmd_rate_x
float64 ommpc_cmd_rate_y
float64 ommpc_cmd_rate_z

float64 ommpc_thr2acc
float64 ommpc_thrust_scale_compensate
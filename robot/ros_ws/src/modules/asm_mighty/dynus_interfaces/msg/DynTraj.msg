std_msgs/Header header
float32[] bbox             #bounding box. It has three elements: hx, hy, hz (bbox of size hx x hy x hz)
int32 id                   #id of the dynamic obstacle
PWPTraj pwp
QuinticPWPTraj quintic_pwp
float32[] ekf_cov_p
float32[] ekf_cov_q
float32[] ekf_cov_r
float32[] poly_cov
float32[] poly_coeffs_x
float32[] poly_coeffs_y
float32[] poly_coeffs_z
float64   poly_start_time
float64   poly_end_time
string[] function #Theis field is only used by the obstacles (i.e. when is_agent==false)
string[] velocity #Theis field is only used by the obstacles (i.e. when is_agent==false)
geometry_msgs/Vector3 pos
bool is_agent # defalut is false for boolean (https://design.ros2.org/articles/interface_definition.html)
float32[] goal
string mode # "analytic" or "pwp" or "poly" or "quintic_pwp"
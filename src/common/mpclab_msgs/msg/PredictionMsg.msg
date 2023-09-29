# This is a message to communicate the agent's prediction
std_msgs/Header     header
float64             t           # Time
float64[]           x           # Global x
float64[]           y           # Global y
float64[]           v_x         # Global v_x
float64[]           v_y         # Global v_y
float64[]           a_y         # Global a_y
float64[]           a_x         # Global a_x
float64[]           psi         # Yaw
float64[]           psidot      # Yaw rate
float64[]           s           # Distance along track
float64[]           x_tran      # distance from track
float64[]           v_long      # Longitudinal velocity
float64[]           v_tran      # Transverse velocity
float64[]           a_long      # Longitudinal acceleration
float64[]           a_tran      # Lateral acceleration
float64[]           e_psi       # Heading deviation
float64[]           u_a         # Acceleration input
float64[]           u_steer     # Steering angle input
float64             lap_num     # Lap number

float64[]           sey_cov
float64[]           xy_cov

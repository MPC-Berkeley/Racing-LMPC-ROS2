# This is a message to hold data with vehicle actuation commands
std_msgs/Header             header

float64                     t         # Time
float64                     u_a       # Acceleration input
float64                     u_steer   # Steering angle input

mpclab_msgs/TimingMsg     timing

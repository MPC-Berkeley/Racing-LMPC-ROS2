# This is a message to hold data with vehicle actuation commands
std_msgs/Header header
float64 t         # Time
int8 exitflag
mpclab_msgs/VehicleActuationMsg  control
float64 x_ref
bool blocking

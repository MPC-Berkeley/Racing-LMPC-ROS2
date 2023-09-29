# This is a message to hold data with system state information
std_msgs/Header                             header
float64                                     t           # Time

mpclab_msgs/PositionMsg                   x

mpclab_msgs/OrientationEulerMsg           e           # Orientation in Euler angles
mpclab_msgs/OrientationQuaternionMsg      q           # Orientation in quaternions
mpclab_msgs/BodyAngularVelocityMsg        w           # Vector of angular velocities
mpclab_msgs/BodyAngularAccelerationMsg    aa          # Vector of angular accelerations
mpclab_msgs/BodyLinearVelocityMsg         v           # local body velocities
mpclab_msgs/BodyLinearAccelerationMsg     a           # local body accelerations

mpclab_msgs/ParametricPoseMsg             p
mpclab_msgs/ParametricVelocityMsg         pt

mpclab_msgs/VehicleActuationMsg           u
mpclab_msgs/DriveStateMsg                 hw

mpclab_msgs/TimingMsg                     timing

float64                                     lap_num     # Lap number
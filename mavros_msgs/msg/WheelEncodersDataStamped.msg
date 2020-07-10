# Stamped wheel odometry message
#
# For streaming timestamped data from FCU wheel encoders (RPM or WHEEL_DISTANCE).

std_msgs/Header header
int64[4] encoder_position
int64[4] qpps
float[4] distance_traveled
float[4] wheel_velocity

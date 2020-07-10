# Stamped wheel odometry message
#
# For streaming timestamped data from FCU wheel encoders (RPM or WHEEL_DISTANCE).

std_msgs/Header header
int64[4] positions
int64[4] pps
int64[4] ppr

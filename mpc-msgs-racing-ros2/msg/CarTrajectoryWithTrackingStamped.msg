# Trajectory for the car with path tracking parameter
# - The time array encodes the time along a trajectory
# - The state array encodes the state trajectory points
# - The control array encodes the control trajectory points

std_msgs/Header header

builtin_interfaces/Time capture_time

float64[] time
CarStateWithTracking[] state
CarControlWithTracking[] control
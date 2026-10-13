# Control input for the car. The following conventions hold:
# - Steer and throttle are normalized in the range [-1, 1], where a positive value
#   means steering to the left and negative value steeing to the right.
#   A negative throttle reverses/brakes the car and a positive value means forward torque.

std_msgs/Header header

float32 steering
float32 throttle

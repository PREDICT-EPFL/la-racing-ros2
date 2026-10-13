# State for the car. The following conventions hold:
# - x and y are the first two axes of a right-handed frame with z pointing up.
# - The heading angle theta is positive in counter-clockwise sense (positive about the position z axis).
# - The turn rate is the time-derivative of the heading angle. Positive turn rate means counter-clockwise turn.

std_msgs/Header header

float64 pos_x
float64 pos_y

float64 vel_x
float64 vel_y

float64 turn_angle
float64 turn_rate

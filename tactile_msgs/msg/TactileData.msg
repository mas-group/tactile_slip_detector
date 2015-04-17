# This messages represents the tactile information of a list of sensors.
#
# The message contains a list of tactile sensors defined by:
#  * the name to identify each sensor uniquely,
#  * the array with the tactile information of each sensor
#    (e.g. pressure values in each tactel)
#  * the number of sensing elements (e.g. tactels) in the X axis of each sensor
#  * the number of sensing elements (e.g. tactels) in the Y axis of each sensor
#
# All arrays in this message have the same size.
# The order of the sensor names matches the order of the tactile information.

Header header

string[] name
tactile_msgs/TactileArray[] tactile_array
int16[] size_x
int16[] size_y
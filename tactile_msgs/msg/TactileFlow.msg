# This messages represents the tactile flow of a list of sensors.
#
# The message contains a list of tactile sensors defined by:
#  * the name to identify each sensor uniquely,
#  * the contact flow in the X axis of each sensor
#  * the contact flow in the Y axis of each sensor
#
# All arrays in this message have the same size.
# The order of the sensor names matches the order of the tactile information.

Header header

string[] name
float64[] flow_x
float64[] flow_y
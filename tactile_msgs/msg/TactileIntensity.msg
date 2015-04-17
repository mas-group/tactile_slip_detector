# This messages represents intensity information for
# a list of tactile sensors.
#
# The message contains a list of tactile sensors defined by:
#  * the name to identify each sensor uniquely,
#  * the intensity (scalar) of each sensor (e.g. mean intensity)
#
# All arrays in this message have the same size.
# The order of the sensor names matches the order of the intensity information.

Header header

string[] name
float64[] data
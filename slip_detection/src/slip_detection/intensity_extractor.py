#!/usr/bin/env python
"""
This module contains a component that publishes tactile
data (e.g. mean intensity) obtained from tactile sensors.

"""
#-*- encoding: utf-8 -*-

import rospy
import numpy
import std_msgs.msg
import tactile_msgs.msg


class IntensityExtractor:
    """
    Publishes tactile data (e.g. mean intensity) obtained
    from tactile sensors.

    """
    def __init__(self):
        # params
        self.supported_tactile_data = [
            'mean_intensity', 'sum_intensity', 'mean_tactel_intensity'
        ]
        self.event = None
        self.tactile_data = None

        # A map between the supported tactile data types and
        # the functions that calculate them.
        self.tactile_functions = {
            self.supported_tactile_data[0]: lambda x: calculate_mean_intensities(x),
            self.supported_tactile_data[1]: lambda x: calculate_sum_intensities(x),
            self.supported_tactile_data[2]: lambda x: calculate_mean_tactel_intensities(x)
        }

        # type of tactile data (as defined in supported tactile data)
        self.tactile_data_type = rospy.get_param('~tactile_data_type', None)

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.03)

        # list with the labels of each observed tactile sensor
        self.tactile_sensors = rospy.get_param('~tactile_sensors', None)

        # publishers
        self.intensity_data = rospy.Publisher(
            '~intensity_data', tactile_msgs.msg.TactileIntensity
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~tactile_data', tactile_msgs.msg.TactileData, self.tactile_data_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the twist.

        """
        self.tactile_data = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'CONFIGURING':
                state = self.configuring_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            rospy.sleep(self.cycle_time)

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.tactile_data:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'RUNNING'
        elif self.event == 'e_stop':
            return 'INIT'
        else:
            return 'IDLE'

    def configuring_state(self):
        """
        Executes the CONFIGURING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.tactile_data_type in self.supported_tactile_data:
            return 'RUNNING'
        else:
            rospy.logwarn(
                "Tactile data type '{0}' is not supported.\nSupported tactile data types "
                "are:\n{1}".format(self.tactile_data_type, self.supported_tactile_data)
            )
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            return 'INIT'
        else:
            intensity_data = self.calculate_intensity_data(self.tactile_data_type)
            self.intensity_data.publish(intensity_data)

            return 'RUNNING'

    def calculate_intensity_data(self, tactile_data_type):
        """
        Calculates the appropriate intensities (e.g. mean) of the tactile sensors,
        based on the specified tactile data type.

        :param tactile_data_type: The type of tactile data to be calculated.
        :type tactile_data_type: str

        :return: The intensities for each tactile sensor.
        :rtype: tactile_msgs.msg.TactileIntensity

        """
        intensity_data = tactile_msgs.msg.TactileIntensity()
        intensity_data.name = self.tactile_sensors
        number_of_sensors = len(intensity_data.name)
        intensity_data.data = [0] * number_of_sensors

        intensities = self.tactile_functions[tactile_data_type](self.tactile_data)

        # todo: check that name matches order..
        for i, pressure in enumerate(intensities):
            intensity_data.data[i] = pressure

        return intensity_data


def calculate_mean_intensities(tactile_data):
    """
    Calculates the mean intensities of the tactile sensors.

    :param tactile_data: The tactile data from the tactile sensors.
    :type tactile_data: tactile_msgs.msg.TactileData

    :return: The mean intensities for each tactile sensor.
    :rtype: tactile_msgs.msg.TactileIntensity

    """
    return [numpy.mean(xx.tactile_array) for xx in tactile_data.tactile_array]


def calculate_sum_intensities(tactile_data):
    """
    Calculates the sum of all tactels' values for each tactile sensor.

    :param tactile_data: The tactile data from the tactile sensors.
    :type tactile_data: tactile_msgs.msg.TactileData

    :return: The summed intensities of each tactile sensor.
    :rtype: tactile_msgs.msg.TactileIntensity

    """
    return [numpy.sum(xx.tactile_array) for xx in tactile_data.tactile_array]


def calculate_mean_tactel_intensities(tactile_data):
    """
    Calculates the mean intensities of only the active tactels.

    :param tactile_data: The tactile data from the tactile sensors.
    :type tactile_data: tactile_msgs.msg.TactileData

    :return: The mean intensities for the active tactels of each tactile sensor.
    :rtype: float[]

    """
    return map(compute_active_mean, tactile_data.tactile_array)


def compute_active_mean(tactile_data):
    """
    Calculates the mean intensities of only the active tactels.

    :param tactile_data: The tactile array from a tactile sensor.
    :type tactile_data: tactile_msgs.msg.TactileArray

    :return: The mean intensities for the active tactels of a tactile sensor.
    :rtype: float

    """
    if numpy.count_nonzero(tactile_data.tactile_array):
        return numpy.sum(tactile_data.tactile_array) / \
            float(numpy.count_nonzero(tactile_data.tactile_array))
    else:
        return 0.0


def main():
    rospy.init_node("intensity_extractor", anonymous=True)
    intensity_extractor = IntensityExtractor()
    intensity_extractor.start()
    rospy.spin()

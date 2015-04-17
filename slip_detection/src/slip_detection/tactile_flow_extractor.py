#!/usr/bin/env python
"""
This module contains a component that measures the tactile flow in a tactile image.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import tactile_msgs.msg
import slip_detection.flow_calculator as flow_calculator


class TactileFlowExtractor(object):
    """
    Measures the tactile flow of a contact in a tactile image.

    """
    def __init__(self):
        # params
        self.event = None
        self.tactile_data = None
        self.flow_calculators = {}

        # a list of the available tactile sensors
        self.sensors = rospy.get_param('~tactile_sensors', None)

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.03)

        # publishers
        self.tactile_flow = rospy.Publisher('~tactile_flow', tactile_msgs.msg.TactileFlow)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~tactile_data', tactile_msgs.msg.TactileData, self.tactile_data_cb
        )

        if self.sensors:
            for index, sensor in enumerate(self.sensors):
                self.flow_calculators[sensor] = index

            # create a flow calculator for each sensor
            for sensor, _ in self.flow_calculators.items():
                self.flow_calculators[sensor] = flow_calculator.FlowCalculator()
        else:
            rospy.logwarn(
                "'{0}' is not a supported type.\nA list of sensors needs to "
                "be specified."
            )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the tactile information.

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

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            return 'INIT'
        else:
            tactile_flows = self.calculate_flows()
            self.tactile_flow.publish(tactile_flows)
            return 'RUNNING'

    def calculate_flows(self):
        """
        Calculates the tactile flow for all sensors.

        :return: The tactile flow for all sensors.
        :rtype: tactile_msgs.msg.TactileFlow

        """
        number_of_sensors = len(self.sensors)

        flows = tactile_msgs.msg.TactileFlow()
        flows.header = self.tactile_data.header
        flows.header.stamp = rospy.Time.now()

        flows.name = [None] * number_of_sensors
        flows.flow_x = [None] * number_of_sensors
        flows.flow_y = [None] * number_of_sensors

        # todo: check if this can be improved
        for name, calculator in self.flow_calculators.items():
            for i, sensor in enumerate(self.tactile_data.name):
                if name == sensor:
                    [flow_x, flow_y] = calculator.calculate_flow(
                        self.tactile_data.tactile_array[i].tactile_array,
                        self.tactile_data.size_x[i], self.tactile_data.size_y[i]
                    )
                    flows.name[i] = sensor
                    flows.flow_x[i] = flow_x
                    flows.flow_y[i] = flow_y

        return flows


def main():
    rospy.init_node('tactile_flow_extractor', anonymous=True)
    tactile_flow_extractor = TactileFlowExtractor()
    tactile_flow_extractor.start()

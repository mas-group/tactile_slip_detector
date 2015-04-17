#!/usr/bin/env python
"""
This module contains a component that converts tactile data from
an SDH tactile sensor to a hardware independent tactile message.

"""
#-*- encoding: utf-8 -*-

import collections
import operator
import rospy
import std_msgs.msg
import schunk_sdh.msg
import tactile_msgs.msg
import slip_detection.slip_detector_utils as slip_utils


class TactileAdapter:
    """
    Converts the tactile data from an SDH sensor to hardware
    independent tactile message.

    """
    def __init__(self):
        # params
        self.event = None
        self.tactile_data_in = None

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.03)

        # mapping between the SDH matrix ID's and the sensors names
        self.tactile_sensors = rospy.get_param('~tactile_sensors', None)

        # publishers
        self.tactile_data_out = rospy.Publisher(
            '~tactile_data_out', tactile_msgs.msg.TactileData
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~tactile_data_in', schunk_sdh.msg.TactileSensor, self.tactile_data_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the tactile data.

        """
        self.tactile_data_in = msg

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
        if self.tactile_data_in and self.tactile_sensors:
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
            tactile_data = self.convert_tactile_data()
            self.tactile_data_out.publish(tactile_data)

            return 'RUNNING'

    def convert_tactile_data(self):
        """
        Converts the SDH sensors' tactile data to an independent tactile message.

        :return: The tactile data for each tactile sensor.
        :rtype: tactile_msgs.msg.TactileData

        """
        tactile_data = tactile_msgs.msg.TactileData()
        tactile_data.header = self.tactile_data_in.header

        sensor_id_map = slip_utils.join_dictionaries(self.tactile_sensors)

        # sort by values (e.g. {sensor_1: 0, sensor_2: 1, sensor_3: 2...)
        sorted_map = collections.OrderedDict(
            sorted(sensor_id_map.items(), key=operator.itemgetter(1))
        )

        number_of_sensors = len(self.tactile_sensors)
        tactile_data.name = [None] * number_of_sensors
        tactile_data.tactile_array = [None] * number_of_sensors
        tactile_data.size_x = [None] * number_of_sensors
        tactile_data.size_y = [None] * number_of_sensors

        # todo: check if this can be improved
        for key in sorted_map:
            for matrix in self.tactile_data_in.tactile_matrix:
                if sorted_map[key] == matrix.matrix_id:
                    tactile_data.name[matrix.matrix_id] = key
                    tactile_data.size_x[matrix.matrix_id] = matrix.cells_x
                    tactile_data.size_y[matrix.matrix_id] = matrix.cells_y

                    xx = tactile_msgs.msg.TactileArray()
                    xx.tactile_array = matrix.tactile_array
                    tactile_data.tactile_array[matrix.matrix_id] = xx

        return tactile_data


def main():
    rospy.init_node("sdh_tactile_adapter", anonymous=True)
    sdh_tactile_adapter = TactileAdapter()
    sdh_tactile_adapter.start()
    rospy.spin()

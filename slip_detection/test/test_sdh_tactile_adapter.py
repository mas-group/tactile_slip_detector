#!/usr/bin/env python
"""
Integration test for the 'sdh_tactile_adapter' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import schunk_sdh.msg
import tactile_msgs.msg

PKG = 'slip_detection'


class TestTactileAdapter(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True
        )
        self.component_input = rospy.Publisher(
            '~component_input', schunk_sdh.msg.TactileSensor
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', tactile_msgs.msg.TactileData, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.component_input.unregister()
        self.component_output.unregister()

    def test_sdh_tactile_interface(self):
        """
        Verifies that the node correctly converts the tactile information
        from a SDH sensor to a hardware independent tactile message.

        """
        # arrays of the expected tactile message
        sensor_1 = tactile_msgs.msg.TactileArray()
        sensor_1.tactile_array = [1, 2, 3, 4, 5, 6]

        sensor_2 = tactile_msgs.msg.TactileArray()
        sensor_2.tactile_array = [7, 7, 7, 7, 7, 7]

        # complete the tactile message
        desired_result = tactile_msgs.msg.TactileData()
        desired_result.name = ['sensor_1', 'sensor_2']
        desired_result.tactile_array = [sensor_1, sensor_2]
        desired_result.size_x = [3, 2]
        desired_result.size_y = [2, 3]

        # matrices of the tactile sensors
        tactile_matrix_0 = schunk_sdh.msg.TactileMatrix()
        tactile_matrix_0.matrix_id = 0
        tactile_matrix_0.cells_x = 3
        tactile_matrix_0.cells_y = 2
        tactile_matrix_0.tactile_array = [1, 2, 3, 4, 5, 6]

        tactile_matrix_1 = schunk_sdh.msg.TactileMatrix()
        tactile_matrix_1.matrix_id = 1
        tactile_matrix_1.cells_x = 2
        tactile_matrix_1.cells_y = 3
        tactile_matrix_1.tactile_array = [7, 7, 7, 7, 7, 7]

        # complete the tactile sensors
        tactile_sensors = schunk_sdh.msg.TactileSensor()
        tactile_sensors.tactile_matrix.append(tactile_matrix_0)
        tactile_sensors.tactile_matrix.append(tactile_matrix_1)

        while not self.wait_for_result:
            self.component_input.publish(tactile_sensors)
            self.event_out.publish('e_start')

        self.assertSequenceEqual(
            self.result.tactile_array, desired_result.tactile_array
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('test_sdh_tactile_adapter')
    rostest.rosrun(PKG, 'test_sdh_tactile_adapter', TestTactileAdapter)

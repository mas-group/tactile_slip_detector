#!/usr/bin/env python
"""
Integration test for the 'intensity_extractor' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import tactile_msgs.msg

PKG = 'slip_detection'


class TestIntensityExtractor(unittest.TestCase):
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
            '~component_input', tactile_msgs.msg.TactileData
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', tactile_msgs.msg.TactileIntensity, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.component_input.unregister()
        self.component_output.unregister()

    def test_mean_intensity(self):
        """
        Verifies that the node correctly outputs the mean intensity
        of a list of tactile sensors.

        """
        desired_result = tactile_msgs.msg.TactileIntensity()
        desired_result.data = [3.5, 7]

        # arrays of the expected tactile message
        sensor_1 = tactile_msgs.msg.TactileArray()
        sensor_1.tactile_array = [1, 2, 3, 4, 5, 6, 0.0, 0.0, 0.0, 0.0, 0.0]

        sensor_2 = tactile_msgs.msg.TactileArray()
        sensor_2.tactile_array = [7, 7, 7, 7, 7, 7, 0.0, 0.0, 0.0, 0.0, 0.0]

        # complete the tactile message
        tactile_data = tactile_msgs.msg.TactileData()
        tactile_data.name = ['sensor_1', 'sensor_2']
        tactile_data.tactile_array = [sensor_1, sensor_2]
        tactile_data.size_x = [3, 2]
        tactile_data.size_y = [2, 3]

        while not self.wait_for_result:
            self.component_input.publish(tactile_data)
            self.event_out.publish('e_start')

        self.assertSequenceEqual(
            self.result.data, desired_result.data
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('test_intensity_extractor')
    rostest.rosrun(PKG, 'test_intensity_extractor', TestIntensityExtractor)

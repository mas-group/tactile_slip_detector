#!/usr/bin/env python
"""
Integration for the 'tactile_flow_extractor' node.

"""

import numpy.testing
import rospy
import unittest
import rostest
import std_msgs.msg
import tactile_msgs.msg

PKG = 'slip_detection'


class TestTactileFlowExtractor(unittest.TestCase):
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
            '~component_output', tactile_msgs.msg.TactileFlow, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.component_input.unregister()
        self.component_output.unregister()

    def test_publish_result(self):
        """
        Verifies that the node publishes a result (i.e. no functionality test).

        """
        tactile_data = tactile_msgs.msg.TactileData()
        tactile_data.name = ['a', 'b', 'c', 'd']
        tactile_data.size_x = [6, 6, 6, 6]
        tactile_data.size_y = [14, 13, 14, 13]

        tactile_array_a = tactile_msgs.msg.TactileArray()
        tactile_array_a.tactile_array = [0] * 84
        tactile_array_b = tactile_msgs.msg.TactileArray()
        tactile_array_b.tactile_array = [0] * 78
        tactile_array_c = tactile_msgs.msg.TactileArray()
        tactile_array_c.tactile_array = [0] * 84
        tactile_array_d = tactile_msgs.msg.TactileArray()
        tactile_array_d.tactile_array = [0] * 78

        tactile_data.tactile_array.append(tactile_array_a)
        tactile_data.tactile_array.append(tactile_array_b)
        tactile_data.tactile_array.append(tactile_array_c)
        tactile_data.tactile_array.append(tactile_array_d)

        desired_result = tactile_msgs.msg.TactileFlow()
        desired_result.flow_x = 0.0
        desired_result.flow_y = 0.0

        while not self.wait_for_result:
            self.component_input.publish(tactile_data)
            self.event_out.publish('e_start')

        numpy.testing.assert_array_almost_equal(
            self.result.flow_x, desired_result.flow_x
        )
        numpy.testing.assert_array_almost_equal(
            self.result.flow_y, desired_result.flow_y
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('test_tactile_flow_extractor')
    rostest.rosrun(PKG, 'test_tactile_flow_extractor', TestTactileFlowExtractor)

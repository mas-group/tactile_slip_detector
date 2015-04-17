#!/usr/bin/env python
"""
Test unit for the functions in the slip_detector_utils.py module.

To run it, type:
py.test [file_name]

"""

import numpy.testing
import tactile_msgs.msg
import slip_detection.slip_detector_utils as slip_detector_utils


def test_calculate_slippage():
    """
    Tests that the 'calculate_slippage' function returns
    correctly the magnitude of a slippage.

    """
    tactile_flow = tactile_msgs.msg.TactileFlow()
    tactile_flow.name = ['a', 'c', 'b']
    tactile_flow.flow_y = [0.5, 7.1, 1.7]
    tactile_flow.flow_x = [2.0, 1.0, 3.0]

    intensity_data = tactile_msgs.msg.TactileIntensity()
    intensity_data.name = ['c', 'a', 'b']
    intensity_data.data = [20, 10, 5]

    # Sum (flow * intensity) / (Sum (intensity))
    desired = 5.1788

    actual = slip_detector_utils.calculate_slippage(tactile_flow, intensity_data)
    numpy.testing.assert_almost_equal(actual, desired, decimal=4)


def test_calculate_slip_indices():
    """
    Tests that the 'calculate_slip_indices' function returns
    correctly the 'slip_index_x' and the 'slip_index_y'.

    """
    convolved_matrix = numpy.array([
        [1, 2, 3, 4, 5],
        [9, 8, 7, 5, 3],
        [3, 1, 6, 9, 4]
    ])

    actual = slip_detector_utils.calculate_slip_indices(convolved_matrix)
    desired = (0.07142857, 0.1142857)
    numpy.testing.assert_almost_equal(actual, desired, decimal=6)


def test_join_dictionaries():
    """
    Tests that the 'join_dictionaries' functions returns a single (joined) dictionary.

    """
    # test with dictionaries of one element
    actual = slip_detector_utils.join_dictionaries([{'x': 1}, {'y': 2}, {'z': 3}])
    desired = {'x': 1, 'y': 2, 'z': 3}
    numpy.testing.assert_equal(actual, desired)

    # test with dictionaries of more than one element
    actual = slip_detector_utils.join_dictionaries([
        {'x': 1, 'w': 4}, {'y': 2, 'r': 5}, {'z': 3, 'k': 6}
    ])
    desired = {'k': 6, 'r': 5, 'w': 4, 'x': 1, 'y': 2, 'z': 3}
    numpy.testing.assert_equal(actual, desired)

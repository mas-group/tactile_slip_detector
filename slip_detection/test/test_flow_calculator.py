#!/usr/bin/env python
"""
Test unit for the flow_calculator.py module.

To run it, type:
py.test [file_name]

"""

import numpy.testing as testing
import slip_detection.flow_calculator as flow_calculator


def test_create_flow_calculator():
    """
    Tests to verify a flow calculator is created correctly.

    """
    my_calculator = flow_calculator.FlowCalculator()
    assert my_calculator.previous_slip_index_x is None
    assert my_calculator.previous_slip_index_y is None
    assert my_calculator.previous_tactile_matrix is None


def test_calculate_flow():
    """
    Test that the flow calculator returns a valid result.

    """
    my_calculator = flow_calculator.FlowCalculator()

    # Empty (square) matrix
    rows = 5
    columns = 5
    tactile_array = [0] * rows * columns
    desired = my_calculator.calculate_flow(tactile_array, rows, columns)
    actual = (0, 0)
    testing.assert_almost_equal(actual, desired)

    # Empty (rectangular) matrix
    rows = 6
    columns = 13
    tactile_array = [0] * rows * columns
    desired = my_calculator.calculate_flow(tactile_array, rows, columns)
    actual = (0, 0)
    testing.assert_almost_equal(actual, desired)

    # Add values to the matrix and keep those values
    # (i.e. the output at the end should be zero)
    tactile_array = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 176, 228, 205, 377,
        573, 438, 663, 925, 1080, 1052, 560, 575, 380, 739, 902, 1004, 573, 748, 132, 320,
        455, 244, 447, 458, 0, 0, 0, 0, 0, 0
    ]
    my_calculator.calculate_flow(tactile_array, rows, columns)

    # this outputs a non-zero value
    desired = my_calculator.calculate_flow(tactile_array, rows, columns)
    actual = (0, 0)
    assert (actual != desired)

    # after the values are kept the same, the flow should be zero
    desired = my_calculator.calculate_flow(tactile_array, rows, columns)
    actual = (0, 0)
    testing.assert_almost_equal(actual, desired)

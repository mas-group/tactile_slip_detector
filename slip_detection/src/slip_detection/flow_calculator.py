#!/usr/bin/env python
"""
This module contains a class to calculate tactile flow of a contact in a tactile image.

"""
#-*- encoding: utf-8 -*-

import numpy
from scipy import signal
import slip_detection.slip_detector_utils as slip_utils


class FlowCalculator(object):
    """
    Calculates the tactile flow of a contact in a tactile image.

    """
    def __init__(self):
        # params
        self.previous_slip_index_x = None
        self.previous_slip_index_y = None
        self.previous_tactile_matrix = None

    def calculate_flow(self, tactile_array, rows, columns):
        """
        Calculates the optical flow in a tactile array using convolution as described
        in [1].


        :param tactile_array: A list (one-dimensional) containing the intensity values
                              of a tactile sensor.
        :type tactile_array: []

        :param rows: The vertical size, in tactels, of the tactile sensor.
        :type rows: int

        :param columns: The horizontal size, in tactels, of the tactile sensor.
        :type columns: int

        :return: The slip vectors in the horizontal (X) and vertical direction (Y).
        :rtype: (float, float)

        [1] Alcazar, J. A., and Barajas, L. G. "Estimating object grasp sliding via
            pressure array sensing." In Robotics and Automation (ICRA), 2012 IEEE
            International Conference on, pp. 1740-1746. IEEE, 2012.

        """
        assert (rows * columns) == len(tactile_array), "Rows and columns do not match array size!"
        tactile_matrix = numpy.array([tactile_array])
        tactile_matrix = tactile_matrix.reshape((rows, columns))

        # convolve the current tactile matrix with the previous tactile matrix
        if self.previous_tactile_matrix is None:
            convolved_matrix = signal.convolve(tactile_matrix, tactile_matrix)
        else:
            convolved_matrix = signal.convolve(
                self.previous_tactile_matrix, tactile_matrix
            )

        # compute slip indices
        [slip_index_x, slip_index_y] = slip_utils.calculate_slip_indices(convolved_matrix)

        # create slip vectors
        if self.previous_slip_index_x:
            slip_vector_x = slip_index_x - self.previous_slip_index_x
        else:
            slip_vector_x = slip_index_x

        if self.previous_slip_index_y:
            slip_vector_y = slip_index_y - self.previous_slip_index_y
        else:
            slip_vector_y = slip_index_y

        # update variables
        self.previous_tactile_matrix = tactile_matrix
        self.previous_slip_index_x = slip_index_x
        self.previous_slip_index_y = slip_index_y

        return slip_vector_x, slip_vector_y

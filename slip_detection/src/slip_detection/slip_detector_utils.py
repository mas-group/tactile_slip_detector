#!/usr/bin/env python
"""
This module contains a collection of functions used in slip detection.

"""
#-*- encoding: utf-8 -*-

import numpy


def calculate_slippage(tactile_flow, tactile_intensity):
    """
    Calculates the amount of slippage on a grasp, based on the
    tactile flow of all sensors and their respective intensity
    (e.g. the higher the intensity on a sensor, the greater the
    'weight' of that sensor's flow).

    slippage = Sum (flow * intensity) / (Sum (intensity))

    :param tactile_flow: The tactile flow in each sensor.
    :type tactile_flow: tactile_msgs.msg.TactileFlow

    :param tactile_intensity: The tactile intensity of each sensor.
    :type tactile_intensity: tactile_msgs.msg.TactileIntensity

    :return: The magnitude of the slippage in the grasp.
    :rtype: float

    """
    # order the sensors' names to match flow and intensity
    intensity = zip(tactile_intensity.name, tactile_intensity.data)
    flow = zip(tactile_flow.name, tactile_flow.flow_x, tactile_flow.flow_y)

    # sort by name, and separate (unzip) names from values (i.e. flow and intensity)
    intensity = zip(*sorted(intensity, key=lambda x: x[0]))
    flow = zip(*sorted(flow, key=lambda x: x[0]))

    # retrieve only the values
    intensity_data = intensity[1]
    flow_x = flow[1]
    flow_y = flow[2]

    # calculate the Euclidean norm of the tactile flow
    flow_norm = numpy.sqrt(numpy.power(flow_x, 2) + numpy.power(flow_y, 2))

    # linear combination of the flow and intensity
    slippage = sum(flow_norm * intensity_data)

    # normalize by the intensity
    if sum(intensity_data):
        return float(slippage) / sum(intensity_data)
    else:
        return 0.0


def calculate_slip_indices(matrix):
    """
    Calculates the slip indices along the X and Y directions,
    based on [1] as:

            Slip index along the X direction (i[T])
                i[T] = AP' / sum(P)
            Slip index along the Y direction (j[T])
                j[T] = Q'B / sum(Q)

    where:
    - E[] denotes the mean value
    - P is the mean value of each column from the matrix C[T]. (row vector)
    - Q is the mean value of each row from the matrix C[T]. (column vector)
    - A and B are offset vectors in the vertical and horizontal directions, respectively.

    :param matrix: The convolved matrix.
    :type matrix: numpy.array

    :return: The slip indices for the X and Y directions.
    :rtype: []float

    [1] Alcazar, J. A., and Barajas, L. G. "Estimating object grasp sliding via
        pressure array sensing." In Robotics and Automation (ICRA), 2012 IEEE
        International Conference on, pp. 1740-1746. IEEE, 2012.

    """
    vector_p = matrix.mean(axis=0)
    vector_q = matrix.mean(axis=1)[numpy.newaxis].T

    vector_a_size = (len(vector_p) + 1) / 2
    vector_b_size = (len(vector_q) + 1) / 2

    vector_a = numpy.array([numpy.arange(-(vector_a_size - 1), vector_a_size, 1)])
    vector_b = numpy.array([numpy.arange(-(vector_b_size - 1), vector_b_size, 1)])
    vector_b = vector_b.T

    if sum(vector_p):
        slip_index_x = numpy.mean(numpy.dot(vector_a, vector_p) / sum(vector_p))
    else:
        slip_index_x = 0.0

    if sum(vector_q):
        slip_index_y = numpy.mean(numpy.dot(vector_q.T, vector_b) / sum(vector_q))
    else:
        slip_index_y = 0.0

    return slip_index_x, slip_index_y


def join_dictionaries(dictionaries_list):
    """
    Joins a list of dictionaries into a single dictionary.

    :param dictionaries_list: A list of dictionaries.
    :type dictionaries_list: []dict

    :return: A single dictionary with all keys and values.
    :rtype: dict

    Example:
        join_dictionaries([{'x': 1}, {'y': 2}, {'z': 3}])
        Out[5]: {'x': 1, 'y': 2, 'z': 3}

    Based on:
    http://stackoverflow.com/questions/3494906/how-do-i-merge-a-list-of-dicts-into-a-single-dict
    """
    joined_dictionaries = {}

    for d in dictionaries_list:
        joined_dictionaries.update(d)

    return joined_dictionaries


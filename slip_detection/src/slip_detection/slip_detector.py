#!/usr/bin/env python
"""
This module contains a component that detects an object slipping from a robot grasp.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import tactile_msgs.msg
import slip_detection.slip_detector_utils as slip_utils


class SlipDetector(object):
    """
    Detects the slippage from a robot grasp

    """
    def __init__(self):
        # params
        self.event = None
        self.tactile_flow = None
        self.intensity_data = None

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.03)
        # threshold used to detect a slippage based on the tactile data
        self.slip_threshold = rospy.get_param('~tactile_threshold', 0.5)

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.tactile_event_out = rospy.Publisher('~tactile_event_out', std_msgs.msg.String)
        self.slip = rospy.Publisher('~slip', std_msgs.msg.Float32)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~tactile_flow', tactile_msgs.msg.TactileFlow, self.tactile_flow_cb
        )
        rospy.Subscriber(
            '~intensity_data', tactile_msgs.msg.TactileIntensity, self.intensity_data_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def tactile_flow_cb(self, msg):
        """
        Obtains the tactile flow.

        """
        self.tactile_flow = msg

    def intensity_data_cb(self, msg):
        """
        Obtains the intensity data.

        """
        self.intensity_data = msg

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
        if self.tactile_flow and self.intensity_data:
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
            slip = self.detect_slip()
            if slip:
                self.slip.publish(slip)
                self.event_out.publish('e_slip')
            else:
                self.slip.publish(0.0)

            return 'RUNNING'

    def detect_slip(self):
        """
        Detects if a slip has occurred.

        :return: The amount of slippage.
        :rtype: float

        """
        slip = slip_utils.calculate_slippage(self.tactile_flow, self.intensity_data)

        if slip >= self.slip_threshold:
            # this is done to get rid of spurious slippage (and then normalize)
            slip -= self.slip_threshold
            return slip


def main():
    rospy.init_node('slip_detector', anonymous=True)
    slip_detector = SlipDetector()
    slip_detector.start()

#! /usr/bin/env python

import rospy
import actionlib
from eleven_msgs.msg import GoToFeedback, GoToResult, GoToAction


class ElevenAction(object):
    _feedback = GoToFeedback()
    _result = GoToResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, GoToAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Execute Goal")
        rospy.loginfo(goal)

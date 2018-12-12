#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class AncubePad(object):
    def __init__(self):
        self.rate = rospy.Rate(10.0)

        self.cmdTopic = rospy.get_param("~cmd_topic", "cmd_vel")
        self.joyTopic = rospy.get_param("~joy_topic", "joy")

        self.pub_cmd = rospy.Publisher(self.cmdTopic,Twist,queue_size=10)
        self.sub_joy = rospy.Subscriber(self.joyTopic,Joy,self.joyCallback, queue_size=10)
        

        
    def joyCallback(self, msg):

        pass
    

    def update(self):
        pass


if __name__ == "__main__":
    try:
        rospy.init_node("ancube_teleop_node")
        rospy.loginfo("Ancube teleop node started.")

        pad = AncubePad()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
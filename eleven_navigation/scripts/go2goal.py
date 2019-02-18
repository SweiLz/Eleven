#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose2D, PoseArray
from sensor_msgs.msg import LaserScan
from ancube_msgs.msg import Path2D
from ancube_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, atan2, sqrt, sin, cos

import angles


class Robot(object):
    def __init__(self):
        rospy.init_node("robot_controller")
        self.cmd_topic = rospy.get_param('~cmd_topic', "cmd_vel")
        self.laser_topic = rospy.get_param('~laser_topic', "scan")
        self.dist_tolerance = float(rospy.get_param('~dist_tolerance', 0.2))
        self.dist_end = float(rospy.get_param('~dist_end', 0.1))
        self.laser_clearance = float(rospy.get_param('~laser_clearance', 0.7))
        self.poseFreq = float(rospy.get_param('~poseFreq', 20))

        self.timer_pose = rospy.Timer(rospy.Duration(
            1.0/self.poseFreq), self.timerPoseCB)
        self.pub_cmd_vel = rospy.Publisher(
            self.cmd_topic, Twist, queue_size=10)
        self.sub_scan = rospy.Subscriber(
            self.laser_topic, LaserScan, self.update_scan)
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)
        self.pose = Pose2D()
        msg = rospy.wait_for_message(self.laser_topic, LaserScan)
        self.scan_filter = [99]

        self.service_move_to = rospy.Service("/move_to", moveTo, self.serviceMoveTo)


    def timerPoseCB(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(
                'map', 'base_footprint', rospy.Time())
            self.pose.x = trans[0]
            self.pose.y = trans[1]
            self.pose.theta = euler_from_quaternion(rot)[2]
            # rospy.loginfo(self.pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def update_scan(self, msg):
        LIDAR_ERR = 0.1
        scan_filter_temp = []
        # rospy.loginfo(len(msg.ranges))
        for i in range(775):
            if msg.ranges[i] >= LIDAR_ERR:
                scan_filter_temp.append(msg.ranges[i])
        self.scan_filter = scan_filter_temp
        # rospy.loginfo(min(self.scan_filter))

    def euclidean_distance(self, poseA, poseB):
        return sqrt(pow(poseA.x - poseB.x, 2)+pow(poseA.y - poseB.y, 2))


    def serviceMoveTo(self, msg):
        rospy.loginfo(msg)
        # rospy.sleep(5)
        self.move2goal(msg.pose)
        return moveToResponse()

    def move2goal(self, goal):

        command = Twist()

        integral_rotate_z = 0

        while self.euclidean_distance(goal, self.pose) >= self.dist_tolerance:

            if min(self.scan_filter) < self.laser_clearance:
                command.linear.x = 0.0
                command.angular.z = 0.0
                rospy.loginfo("Laser Dis")
            else:
                error_x = goal.x - self.pose.x
                error_y = goal.y - self.pose.y

                error_th = atan2(error_y, error_x)
                error_th = angles.shortest_angular_distance(
                    self.pose.theta, error_th)

                integral_rotate_z += error_th

                if abs(error_th) > 0.5:
                    command.linear.x = 0.2
                    command.angular.z = 0.45 * \
                        (error_th) + 0.01 * integral_rotate_z
                    if abs(error_th) > 0.48:
                        integral_rotate_z = 0

                else:
                    command.linear.x = 0.4
                    command.angular.z = 0.40 * \
                        (error_th) + 0.02 * integral_rotate_z
                    # integral_rotate_z = 0
                    # command.linear.x = 0.25
                    # command.angular.z = 0.70 * (error_th) #+ 0.05 * integral_rotate_z
                txt = "{},{}".format(self.euclidean_distance(
                    goal, self.pose), command.angular.z)

                rospy.loginfo(txt)
            self.pub_cmd_vel.publish(command)
            self.rate.sleep()

        command.linear.x = 0.0
        command.angular.z = 0.0
        self.pub_cmd_vel.publish(command)
        rospy.loginfo("Goal Reach")


if __name__ == "__main__":
    try:
        robot = Robot()
        rospy.loginfo("HelloWorld")

        # goal_poses = {
        #     0: (10.1819, -8.3376),
        #     1: (10.5238, -11.7919),
        #     2: (15.7941, -11.1414),
        #     3: (15.1038, -7.323),
        #     4: (19.4293, -6.8259),
        #     5: (22.1219, -6.4217),
        #     6: (14.3713, -2.882),
        #     7: (13.7628, 1.0976),
        #     8: (8.8336, 0.7343),
        #     9: (6.5943, 2.5905),
        #     10: (7.0784, 6.1065),
        #     11: (8.9669, 6.3103),
        #     12: (10.825, 4.7789),
        #     13: (4.8692, 0.629),
        #     14: (1.7604, 0.3867)

        # }

        # path_list = [1,2,3,4,3,2,5,6,5,2,1]
        # path_list = [0,1,2,3,6,7,8,13,14,13,8,7,6,3,2,1]

        # goal_pos = Pose2D()

        # goal_pos.x = goal_poses[4][0]
        # goal_pos.y = goal_poses[4][1]

        # robot.move2goal(goal_pos)
        # while not rospy.is_shutdown():
        #     point = int(raw_input("Enter Point: "))
        #     goal_pos.x = goal_poses[point][0]
        #     goal_pos.y = goal_poses[point][1]

        #     robot.move2goal(goal_pos)

        # while not rospy.is_shutdown():
        # for path in path_list:
        #     goal_pos.x = goal_poses[path][0]
        #     goal_pos.y = goal_poses[path][1]

        #     robot.move2goal(goal_pos)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

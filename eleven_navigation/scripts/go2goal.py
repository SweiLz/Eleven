#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose2D
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, sin, cos

import angles


class Robot(object):
    def __init__(self):
        rospy.init_node("robot_controller")
        self.cmd_topic = rospy.get_param('~cmd_topic', "cmd_vel")
        self.pose_topic = rospy.get_param('~pose_topic', "pose_amcl")
        self.laser_topic = rospy.get_param('~laser_topic', "scan")
        self.dist_tolerance = float(rospy.get_param('~dist_tolerance', 0.2))
        self.laser_clearance = float(rospy.get_param('~laser_clearance', 0.7))

        self.pub_cmd_vel = rospy.Publisher(
            self.cmd_topic, Twist, queue_size=10)
        self.sub_pose = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.update_pose)
        self.sub_scan = rospy.Subscriber(
            self.laser_topic, LaserScan, self.update_scan)

        self.rate = rospy.Rate(10)

        self.pose = Pose2D()
        self.pose_goal = Pose2D()
        msg = rospy.wait_for_message(self.laser_topic, LaserScan)
        self.scan_filter = [99]

    def update_scan(self, msg):
        LIDAR_ERR = 0.08
        scan_filter_temp = []
        # rospy.loginfo(len(msg.ranges))
        for i in range(360):
            if msg.ranges[i] >= LIDAR_ERR:
                scan_filter_temp.append(msg.ranges[i])
        self.scan_filter = scan_filter_temp
        # rospy.loginfo(min(self.scan_filter))

    def update_pose(self, msg):
        self.pose.x = round(msg.pose.pose.position.x, 4)
        self.pose.y = round(msg.pose.pose.position.y, 4)
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        self.pose.theta = round(euler_from_quaternion(quaternion)[2], 4)
        # rospy.loginfo(self.pose)

    def euclidean_distance(self, poseA, poseB):
        return sqrt(pow(poseA.x - poseB.x, 2)+pow(poseA.y - poseB.y, 2))

    def move2goal(self, goal):
        self.pose_goal.x = goal.x
        self.pose_goal.y = goal.y
        self.pose_goal.theta = goal.theta

        command = Twist()

        integral_rotate_z = 0

        while self.euclidean_distance(self.pose_goal, self.pose) >= self.dist_tolerance:

            if min(self.scan_filter) < self.laser_clearance:
                command.linear.x = 0.0
                command.angular.z = 0.0
                rospy.loginfo("Laser Dis")
            else:
                error_x = self.pose_goal.x - self.pose.x
                error_y = self.pose_goal.y - self.pose.y

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
                    self.pose_goal, self.pose), command.angular.z)

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

        goal_poses = {
            0: (10.1819, -8.3376),
            1: (10.5238, -11.7919),
            2: (15.7941, -11.1414),
            3: (15.1038, -7.323),
            4: (19.4293, -6.8259),
            5: (22.1219, -6.4217),
            6: (14.3713, -2.882),
            7: (13.7628, 1.0976),
            8: (8.8336, 0.7343),
            9: (6.5943, 2.5905),
            10: (7.0784, 6.1065),
            11: (8.9669, 6.3103),
            12: (10.825, 4.7789),
            13: (4.8692, 0.629),
            14: (1.7604, 0.3867)

        }

        # path_list = [1,2,3,4,3,2,5,6,5,2,1]
        path_list = [0,1,2,3,6,7,8,13,14,13,8,7,6,3,2,1]

        goal_pos = Pose2D()

        # goal_pos.x = goal_poses[4][0]
        # goal_pos.y = goal_poses[4][1]

        # robot.move2goal(goal_pos)
        # while not rospy.is_shutdown():
        #     point = int(raw_input("Enter Point: "))
        #     goal_pos.x = goal_poses[point][0]
        #     goal_pos.y = goal_poses[point][1]

        #     robot.move2goal(goal_pos)


        while not rospy.is_shutdown():
            for path in path_list:
                goal_pos.x = goal_poses[path][0]
                goal_pos.y = goal_poses[path][1]

                robot.move2goal(goal_pos)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

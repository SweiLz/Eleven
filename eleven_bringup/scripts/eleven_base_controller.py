#! /usr/bin/env python

import math
import sys

import rospy
import serial
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse


class BaseControl(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = long(rospy.get_param("~baud", "115200"))

        self.baseId = rospy.get_param("~base_id", "base_footprint")
        self.odomId = rospy.get_param("~odom_id", "odom")
        self.cmdTopic = rospy.get_param("~cmd_topic", "cmd_vel")
        self.odomTopic = rospy.get_param("~odom_topic", "odom")
        self.odomFreq = float(rospy.get_param("~odom_freq", "20"))
        self.imuTopic = rospy.get_param("~imu_topic", "imu")

        self.isPubOdom = bool(rospy.get_param("~publish_odom_frame","false"))

        self.commandFreq = float(rospy.get_param("~command_freq", "20"))

        self.wheelSep = float(rospy.get_param("~wheel_separation", "0.46"))
        self.wheelRad = float(rospy.get_param("~wheel_radius", "0.08255"))

        self.linear_max = float(rospy.get_param("~linear_max", "0.6"))
        self.linear_min = float(rospy.get_param("~linear_min", "-0.6"))
        self.angular_max = float(rospy.get_param("~angular_max", "0.6"))
        self.angular_min = float(rospy.get_param("~angular_min", "-0.6"))

        self.serial = None
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=10)
            rospy.loginfo("Connect to port: " + self.port + " success!!")
        except Exception as e:
            rospy.logerr("Cannot connect to port: " + self.port)
            rospy.logerr("Error: " + str(e))

        self.sub_cmd = rospy.Subscriber(
            self.cmdTopic, Twist, self.cmdvelCB, queue_size=10)
        self.sub_imu = rospy.Subscriber(
            self.imuTopic, Imu, self.imuCB, queue_size=10)
        self.pub_odom = rospy.Publisher(
            self.odomTopic, Odometry, queue_size=10)

        self.timer_command = rospy.Timer(rospy.Duration(
            1.0/self.commandFreq), self.timerCommandCB)
        self.timer_odom = rospy.Timer(rospy.Duration(
            1.0/self.odomFreq), self.timerOdomCB)

        self.service_reset = rospy.Service("/reset", Empty, self.serviceReset)

        self.transform = tf.TransformBroadcaster()

        self.trans_x = 0.0
        self.goal_trans_x = 0.0
        self.rotat_z = 0.0
        self.rotat_z_imu = 0.0
        self.goal_rotat_z = 0.0

        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        self.previous_time = rospy.Time.now()


    @staticmethod
    def constrain(value, value_min, value_max):
        return max(min(value_max, value), value_min)

    def cmdvelCB(self, msg):
        self.goal_trans_x = self.constrain(
            msg.linear.x, self.linear_min, self.linear_max)
        self.goal_rotat_z = self.constrain(
            msg.angular.z, self.angular_min, self.angular_max)
        
        

    def imuCB(self, msg):
     
        self.rotat_z_imu = msg.angular_velocity.z


        quaternion = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.pose_th = euler[2]

    def timerCommandCB(self, event):

        WL = (self.goal_trans_x - self.wheelSep /
              2.0*self.goal_rotat_z)/self.wheelRad
        WR = -(self.goal_trans_x + self.wheelSep /
              2.0*self.goal_rotat_z)/self.wheelRad

        lWL = self.constrain(int(WL*10) & 0xFF, 0, 255)
        lWR = self.constrain(int(WR*10) & 0xFF, 0, 255)


        command = [255, lWR, lWL, 254]
        if self.serial != None:
            self.serial.write(command)

    def timerOdomCB(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        speedL = self.vel_wheel_l * self.wheelRad
        speedR = -self.vel_wheel_r * self.wheelRad
        
        self.trans_x = round((speedR + speedL) / 2.0,4)
        self.rotat_z = round((speedR - speedL) / self.wheelSep,4)


        self.pose_x += (self.trans_x * math.cos(self.pose_th)*dt)
        self.pose_y += (self.trans_x * math.sin(self.pose_th)*dt)
        # self.pose_th += (self.rotat_z * dt)

        pose_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.pose_th)

        msg = Odometry()
        msg.header.stamp = current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id = self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = self.trans_x
        msg.twist.twist.angular.z = self.rotat_z

        for i in range(36):
            msg.pose.covariance[i] = 0
        msg.pose.covariance[0] = 0.01 #Error X
        msg.pose.covariance[7] = 0.01 #Error Y
        msg.pose.covariance[14] = 99999
        msg.pose.covariance[21] = 99999
        msg.pose.covariance[28] = 99999
        msg.pose.covariance[35] = 0.01 #Error Yaw

        msg.twist.covariance = msg.pose.covariance
        # for i in range(36):
        #     msg.twist.covariance[i] = 0
        # msg.twist.covariance[0] = 0.005  # speedX
        # msg.twist.covariance[8] = 0.0009 # speedY
        # msg.twist.covariance[35] = 0.01  # speedTh
        self.pub_odom.publish(msg)

        if self.isPubOdom:
            self.transform.sendTransform((self.pose_x, self.pose_y, 0), (
            pose_quat[0], pose_quat[1], pose_quat[2], pose_quat[3]), current_time, self.baseId, self.odomId)

    def serviceReset(self, event):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        return EmptyResponse()


if __name__ == '__main__':
    try:
        rospy.init_node("eleven_base_control")
        rospy.loginfo("Eleven Base control ...")
        robot = BaseControl()
        # rospy.spin()
        while not rospy.is_shutdown():
            try:
                if robot.serial.readable():
                    try:
                        if robot.serial.read() == b'\xff':
                            byte = ord(robot.serial.read())
                            vel_wheel_r = (256-byte) * \
                                (-1/10.0) if byte > 127 else byte/10.0
                            # vel_wheel_r = (
                            #     256-byte)/10.0 if byte > 127 else -byte/10.0
                            byte = ord(robot.serial.read())
                            vel_wheel_l = (256-byte) * \
                                (-1/10.0) if byte > 127 else byte/10.0
                            robot.serial.read()
                            robot.vel_wheel_l = vel_wheel_l
                            robot.vel_wheel_r = vel_wheel_r

                            # robot.vel_wheel_r = round(robot.vel_wheel_r*0.8 + vel_wheel_r*0.2,4)
                            # robot.vel_wheel_l = round(robot.vel_wheel_l*0.8 + vel_wheel_l*0.2,4)
                            # comm = ">{},{}<".format(robot.vel_wheel_r,robot.vel_wheel_l)
                            # rospy.loginfo(comm)

                    except Exception:
                        pass

            except KeyboardInterrupt:
                # if robot.serial != None:
                    # robot.serial.close()
                # sys.exit()
                pass

    except rospy.ROSInterruptException:
        pass

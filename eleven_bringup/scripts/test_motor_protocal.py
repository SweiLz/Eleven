#! /usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Twist

class MotorControl(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = long(rospy.get_param("~baud", "115200"))
        self.cmdTopic = rospy.get_param("~cmd_topic", "cmd_vel")

        self.wheelSep = float(rospy.get_param("~wheel_separation", "0.46"))
        self.wheelRad = float(rospy.get_param("~wheel_radius", "0.08255"))


        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=10)
            rospy.loginfo("Connect to port: " + self.port + " success!!")
        except Exception as e:
            rospy.logerr("Cannot connect to port: " + self.port)
            rospy.logerr("Error: " + str(e))

        self.sub_cmd = rospy.Subscriber(
            self.cmdTopic, Twist, self.cmdvelCB, queue_size=10)

        self.goal_trans_x = 0.0
        self.goal_rotat_z = 0.0

        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0

    @staticmethod
    def constrain(value, value_min, value_max):
        return max(min(value_max, value), value_min)
        
    def cmdvelCB(self, msg):
        self.goal_trans_x = msg.linear.x
        self.goal_rotat_z = msg.angular.z

        WL = (self.goal_trans_x - self.wheelSep /
              2.0*self.goal_rotat_z)/self.wheelRad
        WR = -(self.goal_trans_x + self.wheelSep /
              2.0*self.goal_rotat_z)/self.wheelRad

        lWL = self.constrain(int(WL*10) & 0xFF, 0, 255)
        lWR = self.constrain(int(WR*10) & 0xFF, 0, 255)


        command = [255, lWR, lWL, 254]
        if self.serial != None:
            self.serial.write(command)
    

if __name__ == "__main__":
    try:
        rospy.init_node("test_motor_protocal")
        rospy.loginfo("Eleven test motor controller ...")
        robot = MotorControl()
        while not rospy.is_shutdown():
            try:
                if robot.serial.readable():
                    try:
                        if robot.serial.read() == b'\xff':
                            byte = ord(robot.serial.read())
                            vel_wheel_r = (256-byte) * \
                                (-1/10.0) if byte > 127 else byte/10.0
                            byte = ord(robot.serial.read())
                            vel_wheel_l = (256-byte) * \
                                (-1/10.0) if byte > 127 else byte/10.0
                            robot.serial.read()
                            robot.vel_wheel_l = vel_wheel_l
                            robot.vel_wheel_r = vel_wheel_r

                            comm = ">>> {}, {}".format(robot.vel_wheel_r,robot.vel_wheel_l)
                            rospy.loginfo(comm)

                    except Exception:
                        pass

            except KeyboardInterrupt:
                pass
    except rospy.ROSInterruptException:
        pass
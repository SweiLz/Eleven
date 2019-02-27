#!/usr/bin/env python
import sys

import roslaunch
import rospy
import rospkg
import tf


from std_msgs.msg import Float64
from eleven_msgs.srv import changeFloor

map_nav_server_args = "$(find eleven_navigation)/maps/"
map_nav_server_node = roslaunch.core.Node(
    'map_server', 'map_server', name="map_nav_server_node", remap_args=[("map", "map_nav"), ("static_map", "static_map_nav")])

cartographer_args = "-configuration_directory $(find eleven_description)/config -configuration_basename eleven_localization.lua -load_state_filename $(find eleven_localization)/maps/"
cartographer_node = roslaunch.core.Node(
    "cartographer_ros", "cartographer_node", name="cartographer_node")

changeFlag = False
nav_process = None
loc_process = None


def changeFloorCB(req):
    global changeFlag
    rospy.loginfo("Change to floor : {}".format(req.floor))

    map_nav_server_node.args = map_nav_server_args + \
        "fibo_f{}.yaml".format(req.floor)
    cartographer_node.args = cartographer_args + \
        "fibo_f{}.bag.pbstream".format(req.floor)

    changeFlag = True

    # rospy.loginfo(cartographer_node.args)
    # rospy.loginfo(map_nav_server_node.args)
    return True


if __name__ == "__main__":
    rospy.init_node("eleven_manager_node")

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'map', 'base_footprint', rospy.Time(0))
            rospy.loginfo("{:3.4f}, {:3.4f}, {}".format(
                round(trans[0], 4), round(trans[1], 4), tf.transformations.euler_from_quaternion(rot)))
            # rospy.loginfo()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()

    # chFloor = rospy.Service("change_floor", changeFloor, changeFloorCB)
    # alt = rospy.wait_for_message("altitude", Float64)

    # rospy.loginfo(alt)
    # while not rospy.is_shutdown():
    #     if changeFlag == True:
    #         changeFlag = False
    #         nav_process = launch.launch(map_nav_server_node)
    #         loc_process = launch.launch(cartographer_node)

    #     map_nav_server_node.args = "$(find eleven_navigation)/maps/fibo_f2.yaml"
    #     map_process = launch.launch(map_nav_server_node)
    #     loc_process = launch.launch(cartographer_node_f2)

    #     rospy.sleep(5)
    #     map_process.stop()
    #     loc_process.stop()
    #     while loc_process.is_alive():
    #         pass

    #     # rospy.sleep(5)

    #     # cartographer_node.args = "-configuration_directory $(find eleven_description)/config -configuration_basename eleven_localization.lua -load_state_filename $(find eleven_localization)/maps/fibo_f5.bag.pbstream"
    #     map_nav_server_node.args = "$(find eleven_navigation)/maps/fibo_f5.yaml"
    #     map_process = launch.launch(map_nav_server_node)

    #     loc_process2 = launch.launch(cartographer_node_f5)
    #     rospy.sleep(5)
    #     map_process.stop()
    #     loc_process2.stop()
    #     while loc_process.is_alive():
    #         pass

    # rospy.sleep(5)

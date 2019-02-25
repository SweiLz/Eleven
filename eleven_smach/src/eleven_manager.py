#!/usr/bin/env python
import sys

import roslaunch
import rospy
import rospkg


if __name__ == "__main__":
    rospy.init_node("eleven_manager_node")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    map_nav_server_node = roslaunch.core.Node(
        'map_server', 'map_server', name="map_nav_server_node", remap_args=[("map", "map_nav"), ("static_map", "static_map_nav")])

    cartographer_node = roslaunch.core.Node(
        "cartographer_ros", "cartographer_node", name="cartographer_node")

    cartographer_grid_node = roslaunch.core.Node(
        "cartographer_ros", "cartographer_occupancy_grid_node", name="cartographer_occupancy_grid_node", args="-resolution 0.03")

    grid_process = launch.launch(cartographer_grid_node)
    while not rospy.is_shutdown():
        cartographer_node.args = "-configuration_directory $(find eleven_description)/config -configuration_basename eleven_localization.lua -load_state_filename $(find eleven_localization)/maps/fibo_f2.bag.pbstream"
        map_nav_server_node.args = "$(find eleven_navigation)/maps/fibo_f2.yaml"
        map_process = launch.launch(map_nav_server_node)
        loc_process = launch.launch(cartographer_node)

        rospy.sleep(10)
        map_process.stop()
        loc_process.stop()
        rospy.sleep(5)

        cartographer_node.args = "-configuration_directory $(find eleven_description)/config -configuration_basename eleven_localization.lua -load_state_filename $(find eleven_localization)/maps/fibo_f5.bag.pbstream"
        map_nav_server_node.args = "$(find eleven_navigation)/maps/fibo_f5.yaml"
        map_process = launch.launch(map_nav_server_node)
        loc_process = launch.launch(cartographer_node)
        rospy.sleep(10)
        map_process.stop()
        loc_process.stop()
        rospy.sleep(5)

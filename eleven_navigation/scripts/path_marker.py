#!/usr/bin/env python

import os

import roslib
import rospkg
import rospy
from geometry_msgs.msg import Pose2D
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl, Marker)

roslib.load_manifest("interactive_markers")


# Class to manage the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):
    def __init__(self, frame_id, name, description, is_manager=False, speed=0.2):
        InteractiveMarker.__init__(self)
        marker_scale_x = rospy.get_param("~marker_scale_x", 0.2)
        marker_scale_y = rospy.get_param("~marker_scale_y", 0.2)
        marker_scale_z = rospy.get_param("~marker_scale_z", 0.5)

        self.header.frame_id = frame_id
        self.name = name
        self.description = description
        self.speed = speed
        self.marker = Marker()
        self.marker.type = Marker.CYLINDER
        self.marker.scale.x = marker_scale_x
        self.marker.scale.y = marker_scale_y
        self.marker.scale.z = marker_scale_z
        self.marker.pose.position.z = 0.0
        if is_manager:
            self.marker.color.r = 0.8
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.8
        else:
            self.marker.color.r = 0.0
            self.marker.color.g = 0.8
            self.marker.color.b = 0.0
            self.marker.color.a = 0.8

        self.marker_control = InteractiveMarkerControl()
        self.marker_control.always_visible = True
        self.marker_control.orientation.w = 1
        self.marker_control.orientation.x = 0
        self.marker_control.orientation.y = 1
        self.marker_control.orientation.z = 0
        self.marker_control.markers.append(self.marker)
        self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.controls.append(self.marker_control)
        # if not is_manager:
        #     self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # else:
        # self.marker_control.name = "rotate_x"
        #     self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        # self.controls.append(self.marker_control)

    def processFeedback(self, feedback):
        self.pose = feedback.pose


# Manages the creation of waypoints
class PointPathManager(InteractiveMarkerServer):
    def __init__(self, name, frame_id):
        InteractiveMarkerServer.__init__(self, name)
        self.list_of_points = []
        self.frame_id = frame_id
        self.counter_points = 0

        self.menu_handler = MenuHandler()

        entry_waypoints = self.menu_handler.insert("Waypoints")

        entry_creatNew = self.menu_handler.insert(
            "Create New", parent=entry_waypoints)
        self.menu_handler.insert(
            "0.2 m/s", parent=entry_creatNew, callback=self.newPointCB_02)
        self.menu_handler.insert(
            "0.35 m/s", parent=entry_creatNew, callback=self.newPointCB_035)
        self.menu_handler.insert(
            "0.5 m/s", parent=entry_creatNew, callback=self.newPointCB_05)

        self.menu_handler.insert(
            "Delete last", parent=entry_waypoints, callback=self.deletePointCB)
        self.menu_handler.insert(
            "Delete all", parent=entry_waypoints, callback=self.deleteAllPointsCB)
        self.menu_handler.insert(
            "Save", parent=entry_waypoints, callback=self.savePointsCB)
        self.menu_handler.insert(
            "Load", parent=entry_waypoints, callback=self.loadPointsCB)
        self.menu_handler.insert(
            "Go to this point", callback=self.goToPointCB)

        entry_path = self.menu_handler.insert("Path")
        self.menu_handler.insert(
            "Go", parent=entry_path, callback=self.startRouteCB)
        self.menu_handler.insert(
            "Stop", parent=entry_path, callback=self.stopRouteCB)
        self.menu_handler.insert(
            "Go back", parent=entry_path, callback=self.reverseRouteCB)

        self.initial_point = PointPath(
            frame_id, "PointManager", "PointManager", True)
        self.insert(self.initial_point, self.initial_point.processFeedback)

        self.menu_handler.apply(self, self.initial_point.name)
        self.applyChanges()

        rp = rospkg.RosPack()
        self.points_file_path = os.path.join(rp.get_path(
            "eleven_navigation"), "config", "waypoint.txt")

    def createNewPoint(self, speed=0.2):
        name = "p{}".format(self.counter_points)
        new_point = PointPath(self.frame_id, name,
                              "{}->{}".format(name, speed), speed=speed)

        if len(self.list_of_points) > 1:
            new_point.pose.position.x = self.list_of_points[self.counter_points - 1].pose.position.x
            new_point.pose.position.y = self.list_of_points[self.counter_points - 1].pose.position.y
        elif len(self.list_of_points) == 1:
            new_point.pose.position.x = self.list_of_points[0].pose.position.x
            new_point.pose.position.y = self.list_of_points[0].pose.position.y

        new_point.pose.position.x = new_point.pose.position.x + 1.0

        self.list_of_points.append(new_point)
        self.insert(new_point, new_point.processFeedback)
        self.menu_handler.apply(self, name)
        self.applyChanges()
        self.counter_points = self.counter_points+1
        rospy.loginfo("path_marker: Created point {} at {:.4f}, {:.4f}".format(
                new_point.name, new_point.pose.position.x, new_point.pose.position.y))

    def newPointCB_02(self, feedback):
        self.createNewPoint(0.2)

    def newPointCB_035(self, feedback):
        self.createNewPoint(0.35)

    def newPointCB_05(self, feedback):
        self.createNewPoint(0.5)

    def deleteLastPoint(self):
        if self.counter_points > 0:
            p = self.list_of_points.pop()
            self.counter_points = self.counter_points-1
            self.erase(p.name)
            rospy.loginfo("path_marker: Deleted point {}".format(p.name))
        self.applyChanges()

    def deletePointCB(self, feedback):
        self.deleteLastPoint()

    def deleteAllPointsCB(self, feedback):
        for i in range(len(self.list_of_points)):
            self.deleteLastPoint()

    def savePointsCB(self, feedback):
        try:
            file_points = open(self.points_file_path, 'w')
        except IOError, e:
            rospy.logerr("path_marker: {}".format(e))
            return

        for i in self.list_of_points:
            txt = "{};{};{};{};{}\n".format(
                i.name, i.description, round(i.speed, 2), round(i.pose.position.x, 4), round(i.pose.position.y, 4))
            file_points.write(txt)
            rospy.loginfo("path_marker: Saving point {} speed = {:.3f} at {:.4f}, {:.4f}".format(
                i.name, i.speed, i.pose.position.x, i.pose.position.y))
        rospy.loginfo("path_marker: Saved {} points".format(
            len(self.list_of_points)))
        file_points.close()

    def loadPointsCB(self, feedback):
        if self.counter_points > 0:
            for i in range(len(self.list_of_points)):
                self.deleteLastPoint()

        try:
            file_points = open(self.points_file_path, 'r')
        except IOError, e:
            rospy.logerr("path_marker: {}".format(e))
            return

        line = file_points.readline().replace('\n', '')
        while line != '':
            a = line.split(';')

            if len(a) == 5:
                new_point = PointPath(
                    self.frame_id, a[0], a[1], speed=float(a[2]))
                new_point.pose.position.x = float(a[3])
                new_point.pose.position.y = float(a[4])
                rospy.loginfo("path_marker: Loading point {} speed = {:.3f} at {:.4f}, {:.4f}".format(
                    a[0], float(a[2]), float(a[3]), float(a[4])))

                self.list_of_points.append(new_point)
                self.insert(new_point, new_point.processFeedback)
                self.menu_handler.apply(self, a[0])
                self.applyChanges()
                self.counter_points = self.counter_points + 1
            else:
                rospy.logerr(
                    "path_marker: Error processing line {}".format(line))
            line = file_points.readline().replace('\n', '')

        rospy.loginfo("path_marker: Loaded {} points".format(
            self.counter_points))
        file_points.close()

    def goToPointCB(self, feedback):
        pass

    def startRouteCB(self, feedback):
        pass

    def stopRouteCB(self, feedback):
        pass

    def reverseRouteCB(self, feedback):
        pass


if __name__ == "__main__":
    try:
        rospy.init_node("eleven_pathmarker")
        rospy.loginfo("path_marker: ROS Init Node")
        _name = rospy.get_name().replace('/', '')
        _frame_id = rospy.get_param("~frame_id", "map")
        pManager = PointPathManager(_name, frame_id=_frame_id)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("path_marker: ROS interrupt exception")

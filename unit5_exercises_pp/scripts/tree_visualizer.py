#!/usr/bin/env python

"""
Rviz visualization marker for Rapidly Exploring Random Trees (RRT's)
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: February 2021
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import  Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3

class TreeVisualizer:
    def __init__(self, resolution, origin, id=0, frame="map"):
        self.resolution = resolution
        self.origin = origin
        self.id = id
        self.frame = frame
        self.plot_segments = rospy.Publisher('/rrt_tree', Marker, queue_size=1)
        self.segments_marker = Marker()
        self.segments_marker = self.init_marker(self.segments_marker)

    def init_marker(self, marker):
        marker.header = Header(frame_id = self.frame)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Tree"
        marker.id = self.id
        marker.type = Marker.LINE_LIST
        marker.color = ColorRGBA(1, 0, 0, 1)
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.1, 0, 0)
        return marker

    def append(self, node):
        if (node.parent == None):
          return
        else:
          start_point = self.grid_to_world_coord(node.parent.coordinates)
          end_point = self.grid_to_world_coord(node.coordinates)
          self.segments_marker.points.append(Point(*start_point+[0]))
          self.segments_marker.points.append(Point(*end_point+[0]))
          self.plot_segments.publish(self.segments_marker)
          rospy.sleep(0.01)

    def grid_to_world_coord(self, xy_grid):
        world_coordinates = []
        world_coordinates.append(self.resolution * xy_grid[0] + self.origin[0])
        world_coordinates.append(self.resolution * xy_grid[1] + self.origin[1])
        return world_coordinates

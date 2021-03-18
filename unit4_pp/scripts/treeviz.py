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
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

class TreeViz:
    def __init__(self, resolution, origin, id=0, frame="map"):
        self.resolution = resolution
        self.origin = origin
        self.id = id
        self.frame = frame
        self.plot_tree = rospy.Publisher('/rrt_tree', Marker, queue_size=1)
        self.segments_marker = Marker(header = Header(frame_id = self.frame, stamp = rospy.Time.now()),
                          ns = "Tree",
                          id = self.id,
                          type = Marker.LINE_LIST,
                          action = Marker.ADD,
                          scale = Vector3(0.1, 0, 0),
                          color = ColorRGBA(0, 0, 1, 1),
                          pose = Pose(Point(0, 0, 0),Quaternion(x=0., y=0., z=0., w=1.)))
        self.nodes_marker = Marker(header = Header(frame_id = self.frame, stamp = rospy.Time.now()),
                          ns = "Nodes",
                          id = self.id,
                          type = Marker.POINTS, # Marker.SPHERE_LIST
                          action = Marker.ADD,
                          scale = Vector3(0.2, 0.2, 0.0),
                          color = ColorRGBA(0, 0, 1, 1),
                          pose = Pose(Point(0, 0, 0.1),Quaternion(x=0., y=0., z=0., w=1.)))

    def append(self, node):
        if (node.parent == None):
          node_coords = self.grid_to_world_coord(node.coordinates)
          self.nodes_marker.points.append(Point(*node_coords+[0]))
          return
        else:
          start_point = self.grid_to_world_coord(node.parent.coordinates)
          end_point = self.grid_to_world_coord(node.coordinates)
          self.segments_marker.points.append(Point(*start_point+[0]))
          self.segments_marker.points.append(Point(*end_point+[0]))
          self.nodes_marker.points.append(Point(*end_point+[0]))
          while self.plot_tree.get_num_connections() < 1:
            rospy.loginfo_once("Waiting for a connection to Rviz marker publisher...")
          self.plot_tree.publish(self.segments_marker)
          self.plot_tree.publish(self.nodes_marker)

    def grid_to_world_coord(self, xy_grid):
        world_coordinates = []
        world_coordinates.append(self.resolution * xy_grid[0] + self.origin[0])
        world_coordinates.append(self.resolution * xy_grid[1] + self.origin[1])
        return world_coordinates

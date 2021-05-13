#!/usr/bin/env python

"""
Rviz visualization marker for gradient descent path planning on Artificial Potential Fields
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: May 2021
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

class DescentViz:
    def __init__(self, start_index, map_width, resolution, origin, id=0, frame="map"):
        self.start = start_index
        self.previous = start_index
        # number of columns in the occupancy grid
        self.width = map_width
        # side lenght of each grid map cell in meters
        self.resolution = resolution
        # x,y position in grid cell coordinates of the world's coordinate origin
        self.origin = origin
        self.id = id
        self.frame = frame
        self.arrow_counter = 0
        self.nodes_in_path = set()
        self.plot_descent = rospy.Publisher('/gradient_descent', MarkerArray, queue_size=1)
        while self.plot_descent.get_num_connections() < 1:
            rospy.loginfo_once("Waiting for a connection to Rviz marker publisher...")

        self.marker_array_msg = MarkerArray()

        deletion_marker = Marker(header = Header(frame_id = self.frame, stamp = rospy.Time.now()),
                          ns = 'arrows',
                          id = 0,
                          type = Marker.ARROW,
                          action = Marker.DELETEALL)
        self.marker_array_msg.markers.append(deletion_marker)
        self.plot_descent.publish(self.marker_array_msg)
        rospy.sleep(0.2)

    def make_arrow_marker(self, tail, tip, id_num):
        # make a visualization marker 
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = 'arrows'
        m.id = id_num
        m.type = Marker.ARROW
        m.pose.orientation.y = 0
        m.pose.orientation.w = 1
        m.scale = Vector3(0.1, 0.1, 0.1)
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.points = [tail, tip]
        return m

    def draw(self, node, z_value):
        if node not in self.nodes_in_path:
          self.nodes_in_path.add(node)
          if (node == self.start):
            self.previous = self.index_to_world_coord(node)+[z_value * 0.02]
            return
          else:
            self.arrow_counter += 1
            start_point = self.previous
            end_point = self.index_to_world_coord(node)+[z_value * 0.02]
            self.previous = end_point
            arrow = self.make_arrow_marker(Point(*start_point), Point(*end_point), self.arrow_counter)

            self.marker_array_msg.markers.append(arrow)
            self.plot_descent.publish(self.marker_array_msg)

    def index_to_world_coord(self, index):
        """
        Converts a flatmap index value to world coordinates (meters)
        index: a linear index value, specifying a cell/pixel in an 1-D array
        Returns a list containing x,y coordinates in the world frame of reference
        """
        # convert to x,y grid cell/pixel coordinates
        grid_cell_map_x = index % self.width
        grid_cell_map_y = index // self.width
        # convert to world coordinates
        world_coordinates = []
        world_coordinates.append(self.resolution * grid_cell_map_x + self.origin[0])
        world_coordinates.append(self.resolution * grid_cell_map_y + self.origin[1])

        return world_coordinates

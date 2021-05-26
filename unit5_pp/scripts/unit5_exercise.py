#!/usr/bin/env python

"""
ROS node for Artificial Potential Fields
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: Mai 2021
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import copy
import random


def random_force(K = 1.0):
    """
    Toy function that calculates a random force for one grid cell
    returns: a random force value scaled by the constant K
    """
    return K * random.uniform(0, 100)

def populate_attractive_field(height, width, goal_xy):
    """ 
    Creates an attractive field
    returns: 1D flat grid map
    """
    

    pass

def costmap_callback(costmap_msg):
    global attractive_field, repulsive_field, total_field
    total_field = copy.deepcopy(costmap_msg)
    show_text_in_rviz('Ready to accept commands!')
    attractive_field = copy.deepcopy(costmap_msg)
    attractive_field.data = [0] * attractive_field.info.height * attractive_field.info.width


def new_goal_callback(data):
    global attractive_field, repulsive_field, total_field
    # Convert goal position from Rviz into grid cell x,y value
    goal_grid_map = world_to_grid_map(data.pose)
    # Populate attractive field data
    unbounded_attractive_field = populate_attractive_field(attractive_field.info.height, attractive_field.info.width, goal_grid_map)
    # Limit the max value of a grid cell to 100 (max value allowed by nav_msgs/OccupancyGrid msg)
    attractive_field.data = [100 if x > 100 else x for x in unbounded_attractive_field]
    # Publish attractive field
    attractive_field_publisher.publish(attractive_field)
    

def rescale_to_max_value(max_value, input_list):
    """ Rescale each value inside input_list into a target range of 0 to max_value """
    scale_factor = max_value / float(max(input_list))
    # Multiply each item by the scale_factor
    input_list_rescaled = [int(x * scale_factor) for x in input_list]
    return input_list_rescaled

def show_text_in_rviz(text):
    text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(2.0),
                pose=Pose(Point(0.0, 0.0, 2.00), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1.5, 1.5, 1.5),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                text=text)
    text_publisher.publish(text_marker)

def world_to_grid_map(world_pose):
    """ Convert Pose() message in world frame of reference to grid map cell/pixel coordinates """
    grid_cell_x = int((world_pose.position.x - attractive_field.info.origin.position.x) / attractive_field.info.resolution)
    grid_cell_y = int((world_pose.position.y - attractive_field.info.origin.position.y) / attractive_field.info.resolution)
    return [grid_cell_x, grid_cell_y]


if __name__ == '__main__':
    try:
        rospy.init_node('artificial_potential_field_exercise')

        repulsive_field_publisher = rospy.Publisher("repulsive_field", OccupancyGrid, queue_size=10)
        attractive_field_publisher = rospy.Publisher("attractive_field", OccupancyGrid, queue_size=10)
        total_field_publisher = rospy.Publisher("total_field", OccupancyGrid, queue_size=10)
        text_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        costmap_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, costmap_callback)
        goal_subscriber = rospy.Subscriber("move_base_simple/goal", PoseStamped, new_goal_callback)

        attractive_field = OccupancyGrid()
        repulsive_field = OccupancyGrid()
        total_field = OccupancyGrid()

        rospy.loginfo("Artificial Potential Field Exercise Running")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
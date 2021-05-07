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
import copy
import random


def random_attractive_potential(K = 1.0):
    """
    Toy function that calculates a random force for one grid cell
    returns: a random force value scaled by the constant K
    """
    return K * random.uniform(0, 100)

def conical_attractive_potential(current_cell, goal_cell, K = 0.05):
    """
    Calculates the linear attractive force for one grid cell with respect to the target
    current_cell: a list containing x and y values of one map grid cell
    goal_cell: a list containing x and y values of the target grid cell
    K: potential attractive constant
    returns: linear attractive force scaled by the potential attractive constant
    """
    dx = goal_cell[0] - current_cell[0]
    dy = goal_cell[1] - current_cell[1]
    distance = (dx ** 2 + dy ** 2)**0.5
    return K * distance

def quadratic_attractive_potential(current_cell, goal_cell, K = 0.05):
    """
    Calculates the quadratic attractive force for one grid cell with respect to the target
    current_cell: a list containing x and y values of one map grid cell
    goal_cell: a list containing x and y values of the target grid cell
    K: potential attractive constant
    returns: quadratic attractive force scaled by the potential attractive constant
    """
    dx = goal_cell[0] - current_cell[0]
    dy = goal_cell[1] - current_cell[1]
    distance = (dx ** 2 + dy ** 2)**0.5
    quadratic_attractive_force = distance**2
    return K * quadratic_attractive_force

def rescale_to_max_value( max_value, input_list):
    """ Rescale each value into a target range of 0 to max_value """
    scale_factor = max_value / float(max(input_list))
    # Multiply each item by the scale_factor
    input_list_rescaled = [int(x * scale_factor) for x in input_list]
    return input_list_rescaled

def populate_attractive_field(height, width, goal_xy):
    field = [0] * height * width
    for row in range(height):
      for col in range(width):
        # force_value = random_attractive_potential()
        # force_value = conical_attractive_potential([row, col], goal_xy)
        force_value = quadratic_attractive_potential([row, col], goal_xy)

        # Assign to potential field
        field[row + width * col] = force_value
    # Rescale into a target range of [0-100]
    return rescale_to_max_value(100, field)

def callback(msg):
    global attractive_field
    attractive_field = copy.deepcopy(msg)


def world_to_grid_map(world_pose):
    """ Convert Pose() message in world frame of reference to grid map cell/pixel coordinates """
    grid_cell_x = int((world_pose.position.x - attractive_field.info.origin.position.x) / attractive_field.info.resolution)
    grid_cell_y = int((world_pose.position.y - attractive_field.info.origin.position.y) / attractive_field.info.resolution)
    return [grid_cell_x, grid_cell_y]

def new_goal_callback(data):
    global attractive_field
    # Convert goal position from Rviz into grid cell x,y value
    goal_grid_map = world_to_grid_map(data.pose)
    # Calculate attractive field
    attractive_field.data = populate_attractive_field(attractive_field.info.height, attractive_field.info.width, goal_grid_map)

    # Publish attractive field
    attractive_field_publisher.publish(attractive_field)


if __name__ == '__main__':
    try:
        rospy.init_node('artificial_potential_field_exercise')

        attractive_field_publisher = rospy.Publisher("attractive_field", OccupancyGrid, queue_size=10)
        costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback)
        goal_subscriber = rospy.Subscriber("move_base_simple/goal", PoseStamped, new_goal_callback)

        attractive_field = OccupancyGrid()

        rospy.loginfo("Artificial Potential Field Exercise Running")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
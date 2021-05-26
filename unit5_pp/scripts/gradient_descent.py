#! /usr/bin/env python

"""
ROS Artificial Potential Fields gradient descent path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: May 2021
Usage: roslaunch unit5_pp unit5_exercise.launch run_gradient_descent:=true
"""

import rospy


def euclidean_distance(index_a, index_b, map_width):
    """
    Euclidean distance between grid cells provided as linear indexes on a flat map
    """
    a = indexToGridCell(index_a, map_width)
    b = indexToGridCell(index_b, map_width)

    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def indexToGridCell(flat_map_index, map_width):
    """
    Converts a linear index of a flat map to grid cell coordinate values
    flat_map_index: a linear index value, specifying a cell/pixel in an 1-D array
    map_width: the map's width 
    returns: list with [x,y] grid cell coordinates
    """
    grid_cell_map_x = flat_map_index % map_width
    grid_cell_map_y = flat_map_index // map_width
    return [grid_cell_map_x, grid_cell_map_y]

def gradient_descent(start_index, goal_index, width, height, potential_field_data, descent_viz):
  ''' 
  Performs gradient descent on an artificial potential field 
  with a given start, goal node and a total potential field
  '''


  pass


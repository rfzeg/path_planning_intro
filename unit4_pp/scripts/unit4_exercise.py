#! /usr/bin/env python

"""
ROS service server for Rapidly Exploring Random Trees (RRT) algorithm path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: December 2020
Usage: roslaunch unit4_pp unit4_exercise.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
# for Rviz visualization
from tree_visualizer import TreeVisualizer

from math import hypot, sqrt, atan2, cos, sin
from random import randrange as rand
#  Bresenham ray tracing on a grid map
from bresenham import bresenham

# Node class
class Node:
  def __init__(self,coordinates,parent):
    # coordinates: list with [x,y] values of grid cell coordinates
    self.coordinates = coordinates
    # parent: Node object
    self.parent = parent

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a PathPlanningPluginResponse
  '''
  # This is the data you get from the request
  # The costmap is a 1-D tuple flat map representation
  map = list(req.costmap_ros)
  # Change values on the map from unknown to free space
  map[map==255] = 1

  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # To-Do: replace numerical constants preferably with parameters / static_map srv
  # side of each grid map square in meters
  map_resolution = 0.1
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  map_origin = [-20.0, -20.0]

  initial_position = indexToGridCell(start_index, width)
  target_position = indexToGridCell(goal_index, width)

  viz = TreeVisualizer(map_resolution, map_origin, id=1, frame="map")

  # time statistics
  start_time = rospy.Time.now()

  # Calculate a path using RRT
  path = rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, viz)

  if not path:
    rospy.logwarn("No path returned by RRT")
    rospy.loginfo('RRT: Empty path sent to navigation stack')
    path = []
  else:
    # print time statistics
    execution_time = rospy.Time.now() - start_time
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    rospy.loginfo('RRT: Path sent to navigation stack')

  # convert [x,y] points into 1-D linear array index
  path_as_indices = []
  for cell in path:
      # access an element in a 1D-array (map) providing index = x + width*y
      path_as_indices.append(cell[0]+width*cell[1])

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path_as_indices
  rospy.loginfo('RRT: Path sent to navigation stack')

  return resp

def calculateDistance(from_node, to_node):
  """
  Calculates distance between two nodes.
  @param from_node: the node from which to check (list containing x and y values)
  @param to_node: the node to which to check (list containing x and y values)
  @return: the distance
  """
  dx = to_node[0] - from_node[0]
  dy = to_node[1] - from_node[1]
  distance = sqrt(dx ** 2 + dy ** 2)
  return distance

def calculateAngle(from_node, to_node):
  """
  Calculates the angle of a straight line between two nodes.
  @param from_node: the node from which to check (list containing x and y values)
  @paramto_node: the node to which to check (list containing x and y values)
  @return: the angle
  """
  dx = to_node[0] - from_node[0]
  dy = to_node[1] - from_node[1]
  theta = atan2(dy, dx)
  return theta

def indexToGridCell(array_index, map_width):
  """
  Converts a linear index value to a list containing [x,y] grid cell coordinate values
  This transformation is derived from the map width
  @param a linear index value, specifying a cell/pixel in an 1-D array
  @param map_width 
  @return list with [x,y] grid cell coordinates
  """
  grid_cell_map_x = array_index % map_width
  grid_cell_map_y = array_index // map_width
  return [grid_cell_map_x, grid_cell_map_y]

def collision_detected(p1, p2, map, width):
  """
  Test if two nodes are separated by an obstacle by tracing a line between them
  """
  # Compute cells covered by the line p1-p2 using the Bresenham ray tracing algorithm
  covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
  # Check if any of the cells is an obstacle cell
  for cell in covered_cells:
      # access an element in a 1D-array (map) providing index = x + width*y
      if map[cell[0]+width*cell[1]]:
          # Detects a collision if map has a 1
          return True
  # No collision
  return False

def find_closest_node(random_xy_pt, node_list):
  """
  Returns the closest node in the tree
  """
  ## add your code ##
  pass

def create_branch(from_xy, to_xy, max_distance):
  """
  Calculates the x,y values for a new node at the max_branch_distance towards the random point
  """
  ## add your code ##
  pass

def test_goal(current, goal, tolerance):
  """
  Tests if goal has been reached considering a tolerance distance
  """
  ## add your code ##
  pass

def build_path(latest_node):
  """
  Reconstruct the path from the last node added until the start node is reached
  """
  ## add your code ##
    
  pass

def rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, tree_viz):
  ''' 
  Performs Rapidly exploring random trees (RRT) algorithm on a costmap with a given start and goal node
  '''

  ### Add code from exercise 3.5.1. HERE ###

  pass

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
rospy.spin()

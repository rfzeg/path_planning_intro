#! /usr/bin/env python

"""
ROS Rapidly-Exploring Random Trees (RRT) path planning algorithm exercise solution
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: December 2020
Usage: roslaunch unit4_pp unit4_solution.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist

from math import atan2, cos, sin
from random import randrange as rand

from bresenham import bresenham
from treeviz import TreeViz

# Node class
class Node:
  def __init__(self, coordinates, parent=None):
    # coordinates: list with [x,y] values of grid cell coordinates
    self.coordinates = coordinates
    # parent: Node object
    self.parent = parent

def make_plan(req):
  ''' 
  Callback function used by the service server to process requests.
  It returns a PathPlanningPluginResponse
  '''
  # Convert costmap from a 1-D tuple flat map representation
  map = list(req.costmap_ros)
  # Change values on the map from unknown to free space
  map[map==255] = 1
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  map_resolution = 0.1
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  map_origin = [-20.0, -20.0]

  initial_position = indexToGridCell(start_index, width)
  target_position = indexToGridCell(goal_index, width)
  viz = TreeViz(map_resolution, map_origin, id=1, frame="map")

  # time statistics
  start_time = rospy.Time.now()

  # Calculate a path using RRT
  path = rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, viz)

  if not path:
    rospy.logwarn("No path returned by RRT")
    rospy.loginfo('RRT: Empty path sent to navigation stack')
    return_path = []
  else:
    # print time statistics
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('+++++++++++ RRT execution metrics ++++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")

    return_path = []
    for cell in path:
        return_path.append(cell[0]+width*cell[1])

  resp = PathPlanningPluginResponse()
  resp.plan = return_path
  rospy.loginfo('RRT: Path sent to navigation stack')
  
  return resp

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

def calculate_distance(p1, p2):
  """
  Calculates distance between two [x,y] coordinates.
  p1: the point (as list containing x and y values) from which to measure 
  p2: the point (as list containing x and y values) to which to measure
  returns: the distance
  """
  dx = p2[0] - p1[0]
  dy = p2[1] - p1[1]
  distance = (dx ** 2 + dy ** 2)**0.5
  return distance

def calculate_angle(p1, p2):
  """
  Calculates the angle of a straight line between two [x,y] coordinates.
  p1: the point (as list containing x and y values) from which to measure 
  p2: the point (as list containing x and y values) to which to measure
  returns: the angle in radians
  """
  dx = p2[0] - p1[0]
  dy = p2[1] - p1[1]
  theta = atan2(dy, dx)
  return theta

def collision_detected(p1, p2, map, map_width):
  """
  Test if two nodes are separated by an obstacle by tracing a line between them
  p1: the point (as list containing x and y values) from which the check starts
  p2: the point (as list containing x and y values) at which the check ends
  map: the map containing free space and obstacles
  map_width: the map's width
  returns: True if a collision is detected, False if no collision is found
  """
  # Compute cells covered by the line p1-p2 using the Bresenham ray tracing algorithm
  covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
  # Check if any of the cells is an obstacle cell
  for cell in covered_cells:
      # Access an element in a 1D-array (map) providing index = x + map_width*y
      if map[cell[0]+map_width*cell[1]]:
          # Detects a collision if map has a 1
          return True
  # No collision
  return False

def find_closest_node(random_pt, node_list):
  """
  Finds the closest node in the tree
  random_pt: a [x,y] point (as a list)
  node_list: list that keeps all nodes in the tree
  returns: Node instance that is the closest node in the tree
  """
  nearest_distance = float('inf')
  for n in node_list:
    current_distance = calculate_distance(random_pt, n.coordinates)
    if current_distance < nearest_distance:
      nearest_node = n
      nearest_distance = current_distance
  return nearest_node

def create_new_branch_point(p1, p2, max_distance):
  """
  Creates a new point at the max_distance towards a second point
  p1: the point to go from (as a list containing x and y values)
  p2: the point to go to (as a list containing x and y values)
  max_distance: the expand distance (in grid cells)
  returns: new point as a list containing x and y values
  """
  new_point = list(p1)
  d = calculate_distance(new_point, p2)
  theta = calculate_angle(new_point, p2)

  # if distance to closest node is less than the maximum branch lenght
  if max_distance > d:
      max_distance = d

  new_point[0] += int(max_distance * cos(theta))
  new_point[1] += int(max_distance * sin(theta))

  return new_point

def test_goal(p1, p_goal, tolerance):
  """
  Test if goal has been reached considering a tolerance distance
  p1: a [x,y] point (as a list) from where to test
  p_goal: a [x,y] point (as a list) corresponding to the goal
  tolerance: distance margin (in grid cells) allowed around goal
  returns: True goal is within tolerance, False if goal is not within tolerance
  """
  distance = calculate_distance(p1, p_goal)
  if (tolerance > distance):
    return True
  else:
    return False

def rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, tree_viz):
  ''' 
  Performs Rapidly exploring random trees (RRT) algorithm on a costmap with a given start and goal node
  '''
  # Create the root node
  root_node = Node(initial_position)
  # A list to keep all nodes in the tree
  nodes = [root_node]
  # Iteration counter
  interations = 0
  # Iterations limit
  max_iterations = 10000
  # Incremental distance (for simplicity value is expressed in grid cells)
  max_branch_lenght = 20
  # Tolerance margin allowed around target position (value in grid cells)
  goal_tolerance = 20
  # A list to hold the output path from start to goal
  path = []

  while True:
    # Increment the number of iterations
    interations += 1
    # Check if we have exeeded max iterations
    if interations > max_iterations:
      rospy.logwarn("RRT: Max iterations exceeded")
      return path
    # Generate a new random point anywhere in the map
    random_point = [rand(width), rand(height)]

    # Find the closest node
    closest_node = find_closest_node(random_point, nodes)

    # Create a new point at the max. branch distance towards the random point
    candidate_point = create_new_branch_point(closest_node.coordinates, random_point, max_branch_lenght)

    # Verifiy that the new branch is collision free
    if not collision_detected(closest_node.coordinates, candidate_point, map, width):
      # Create a new instance of a Node object and add it to the tree
      latest_node = Node(candidate_point, closest_node)
      nodes.append(latest_node)

      # Optional: Visualize tree graph in Rviz
      tree_viz.append(latest_node)

      # Check if the goal has been reached
      if test_goal(latest_node.coordinates, target_position, goal_tolerance):
        rospy.loginfo('RRT: Goal reached')
        break

  rospy.loginfo('RRT: Path search ended')

  # Reconstruct path by working backwards from target
  path.append(target_position)
  node = latest_node
  while node.parent:
      path.append(node.coordinates)
      node = node.parent
  path.append(node.coordinates)
  # Reverse list
  path.reverse()
  rospy.loginfo('RRT: Done reconstructing path')

  return path

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('rrt_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)
  
  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
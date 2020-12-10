#! /usr/bin/env python

"""
ROS service server for Rapidly exploring random trees (RRT) algorithm path planning exercise
Author: Roberto Zegers R.
Date: December 2020
Usage: roslaunch unit5_exercises_pp unit_5.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse

from math import hypot, sqrt, atan2, cos, sin
from random import randrange as rand
#  Bresenham ray tracing on a grid map
from bresenham import bresenham


# Node class
class Node:
  def __init__(self,coordinates,parent):
    self.coordinates = coordinates
    self.parent = parent

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a PathPlanningPluginResponse
  '''
  # This is the data you get from the request
  # The costmap is a 1-D array version of the original costmap image
  costmap = list(req.costmap_ros)
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal

  # Calculate a path using RRT
  path = rrt(start_index, goal_index, width, height, costmap)

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('RRT: Path send to navigation stack')
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
  Converts a linear index value to a x,y grid cell coordinate value
  This transformation is derived from map width
  @param a linear index value, specifying a cell/pixel in an 1-D array
  @param map_width 
  @return x,y grid cell coordinates
  """
  grid_cell_map_x = array_index % map_width
  grid_cell_map_y = array_index // map_width
  return grid_cell_map_x, grid_cell_map_y

def create_branch(from_node, to_node, max_distance):
  """
  Creates a node at the max_branch_distance towards the random point
  @param from_node: the node to go from (list containing x and y values)
  @param to_node: the node to go to (list containing x and y values)
  @param max_distance: the expand distance
  @return: new node as a list containing x and y values
  """
  new_node = list(from_node)
  d = calculateDistance(new_node, to_node)
  theta = calculateAngle(new_node, to_node)

  # if distance to closest node is less than the maximum branch lenght
  if max_distance > d:
      max_distance = d

  new_node[0] += int(max_distance * cos(theta))
  new_node[1] += int(max_distance * sin(theta))

  return new_node

def find_closest_node(random_pt, node_list):
  # Return the closest node in the tree
  nearest_distance = float('inf')
  for n in node_list:
    current_distance = calculateDistance(random_pt,n.coordinates)
    if current_distance < nearest_distance:
      nearest_node = n
      nearest_distance = current_distance
  return nearest_node

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

def test_goal(p1, goal, tolerance):
  """
  Test if the target point has been reached regarding a certain error on both x and y axes
  """
  if (goal[0]-tolerance<p1[0]<goal[0]+tolerance) and (goal[1]-tolerance<p1[1]<goal[1]+tolerance):
    return True
  else:
    return False

def build_path(latest_node):
  rospy.loginfo("Building path...")
  constructed_path = []
  current_node = latest_node
  while current_node.parent:
    constructed_path.append(current_node.coordinates)
    current_node = current_node.parent
  constructed_path.append(current_node.coordinates)
  constructed_path.reverse()
  return constructed_path

def rrt(start_index, goal_index, width, height, costmap):
  ''' 
  Implementation of Rapidly exploring random trees (RRT) algorithm
  '''
  map_size = height * width
  start_x, start_y = indexToGridCell(start_index, width)
  target_x, target_y = indexToGridCell(goal_index, width)
  initial_position = (start_x, start_y)
  target_position = (target_x, target_y)
  # map is a 1-D array costmap
  map = costmap[:]
  # Change the values on the map from the 0-255 range to between 0 and 1
  map[map==255] = 1
  path=[]

  ### Add code from exercise 3.5.1. HERE ###
  goal_reached = False
  root_node = Node(initial_position,None)
  # list to keep tree of nodes and parents
  nodes = [root_node]
  interations = 0
  max_iterations = 100000
  # Incremental distance
  max_branch_lenght = 20
  # Tolerance margin allowed around target position
  goal_tolerance = 10

  ### Add code from exercise 3.5.2. HERE ###
  while not goal_reached:
    # Increment the number of iterations
    interations += 1
    # Check if we have exeeded max iterations
    if interations > max_iterations:
      rospy.logwarn("max iterations exceeded")
      break
    # Generate a new random point anywhere in the map
    random_point = (rand(width),rand(height))

    ### FIND THE CLOSEST NODE ###
    closest_node = find_closest_node(random_point, nodes)

    ## CREATE A NODE AT THE MAX_BRANCH_DISTANCE TOWARDS THE RANDOM POINT
    newest_node = create_branch(closest_node.coordinates, random_point, max_branch_lenght)

    ### VERIFIY THAT THE NEW BRANCH IS COLLISION FREE ###
    if not collision_detected(closest_node.coordinates, newest_node, map, width):
      # add to the tree
      new_node = Node(newest_node,closest_node)
      nodes.append(new_node)

      # CHECK IF THE GOAL HAS BEEN REACHED
      if test_goal(newest_node,target_position, goal_tolerance):
        goal_reached = True

  # Build the path
  path = build_path(new_node)

  # convert (x,y) points into 1-D linear array index
  output = []
  for cell in path:
      # access an element in a 1D-array (map) providing index = x + width*y
      output.append(cell[0]+width*cell[1])

  # Publish ROS path
  '''
  path_msg = pathPoses_building(path)
  pub_path.publish(path_msg)
  rospy.loginfo("Published msg of type 'Path' to topic '/path'")
  '''
  
  return output

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

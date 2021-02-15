#! /usr/bin/env python

"""
ROS service server for Rapidly Exploring Random Trees (RRT) algorithm path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License License BSD-3-Clause
Date: December 2020
Usage: roslaunch unit5_exercises_pp unit5_exercise.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
# for Rviz visualization
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import  Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3

from math import hypot, sqrt, atan2, cos, sin
from random import randrange as rand
#  Bresenham ray tracing on a grid map
from bresenham import bresenham

def grid_to_world(xy_grid, map_resolution, origin):
    """
    Convert x,y grid cell/pixel coordinates to world coordinates (in meters)
    This transformation is derived from the map resolution and map origin
    @param xy_grid cell/pixel coordinates as a list [x,y]
    @param map_resolution side of each grid map square in meters
    @param origin cell/pixel coordinates as a list [x,y]
    @return list with [x,y] values in world coordinates
    """
    output_list = []
    output_list.append(map_resolution * xy_grid[0] + origin[0])
    output_list.append(map_resolution * xy_grid[1] + origin[1])
    return output_list

def visualize_tree(tree_in_grid_frame, resolution, origin, id=0, frame="map"):
  tree_in_world_frame = []
  for n in tree_in_grid_frame:
    tree_in_world_frame.append(grid_to_world(n.coordinates, resolution, origin))
    tree_in_world_frame.append(grid_to_world(n.parent.coordinates, resolution, origin))

  marker = Marker()
  marker.header = Header(frame_id=frame)
  marker.header.stamp = rospy.Time.now()
  marker.ns = "Tree"
  marker.id = id
  marker.type = Marker.LINE_LIST
  marker.color = ColorRGBA(1, 0, 0, 1)
  marker.pose.position.x = 0
  marker.pose.position.y = 0
  marker.pose.position.z = 0
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.points = [Point(n[0], n[1], 0) for n in tree_in_world_frame]
  marker.scale = Vector3(0.1, 0, 0)

  pub_tree.publish(marker)
  rospy.sleep(0.01)

  return

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
  # The costmap is a 1-D array version of the original costmap image
  costmap = list(req.costmap_ros)
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # To-Do: replace numerical constants preferably with parameters / static_map srv
  # side of each grid map square in meters
  map_resolution = 0.1
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  map_origin = [-20.0, -20.0]

  # time statistics
  start_time = rospy.Time.now()

  # Calculate a path using RRT
  path = rrt(start_index, goal_index, width, height, costmap, map_resolution, map_origin)

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

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  
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

def rrt(start_index, goal_index, width, height, costmap, map_resolution, map_origin):
  ''' 
  Performs Rapidly exploring random trees (RRT) algorithm on a costmap with a given start and goal node
  '''
  map_size = height * width
  start_x, start_y = indexToGridCell(start_index, width)
  target_x, target_y = indexToGridCell(goal_index, width)
  initial_position = [start_x, start_y]
  target_position = [target_x, target_y]
  # map is a 1-D array costmap
  map = costmap[:]
  # Change values on the map from unknown to free space
  map[map==255] = 1

  ### Add code from exercise 3.5.1. HERE ###

  pass

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
pub_tree = rospy.Publisher('/rrt_tree', Marker, queue_size=1)
rospy.spin()

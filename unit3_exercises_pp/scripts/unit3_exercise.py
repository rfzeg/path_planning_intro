#! /usr/bin/env python

"""
ROS service server for Dijkstra's algorithm path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License License BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit3_exercises_pp unit3_exercise.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
import math

rospy.init_node('dijkstra_path_planning_service_server', log_level=rospy.INFO, anonymous=False)

def find_neighbors(index, width, height, costmap, orthogonal_movement_cost):
  """
  Identifies neighbor nodes in free space and inside the map boundaries
  Returns a main list with neighbour nodes as [index, step_cost] pairs (sublists)
  orthogonal_movement_cost: the cost of moving to an adjacent grid cell, the size/edge of one grid cell
  """
  # temporary list
  check = []
  # output list
  neighbors = []

  # check that upper neighbour is not past the map's boundaries
  if index - width > 0:
    # append [index, step_cost] pair
    check.append([index - width, orthogonal_movement_cost])

  # check that left neighbour is not past the map's boundaries
  if (index - 1) % width > 0:
    check.append([index - 1, orthogonal_movement_cost])

  # check that right neighbour is not past the map's boundaries
  if (index + 1) % width != (width + 1):
    check.append([index + 1, orthogonal_movement_cost])

  # check that lower neighbour is not past the map's boundaries
  if (index + width) <= (height * width):
    check.append([index + width, orthogonal_movement_cost])

  for element in check:
     # Check if neighbour node is an obstacle
    if costmap[element[0]] == 0:
      # appends [index, step_cost] sublist to output list
      neighbors.append(element)

  return neighbors


def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  '''
  # costmap as 1-D array representation
  costmap = req.costmap_ros
  # number of columns in the occupancy grid
  width = req.width
  # number of rows in the occupancy grid
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  resolution = 0.2
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  origin = [-7.4, -7.4, 0]

  # time statistics
  start_time = rospy.Time.now()

  # calculate the shortes path using Dijkstra
  path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin)

  if not path:
    rospy.logwarn("No path returned by Dijkstra's shortes path algorithm")
    path = []
  else:
    # print time statistics
    execution_time = rospy.Time.now() - start_time
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin):
  ''' 
  Performs Dijkstra's shortes path algorithm search on a costmap with a given start and goal node
  '''

  #### To-do: complete all exercises below ####

    
    
  pass

# Creates service 'make_plan', service requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

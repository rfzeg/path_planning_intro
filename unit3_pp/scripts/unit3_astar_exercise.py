#! /usr/bin/env python

"""
ROS service server for A-Star's algorithm path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit3_pp unit3_astar_exercise.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
import math

rospy.init_node('astar_path_planning_service_server', log_level=rospy.INFO, anonymous=False)

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

def euclidean_distance(a, b):
    distance = 0
    for i in range(len(a)):
        distance += (a[i] - b[i]) ** 2
    return distance ** 0.5

def manhattan_distance(a, b):
    return (abs(a[0] - b[0]) + abs(a[1] - b[1]))

def indexToWorldCoord(array_index, map_width, map_resolution, map_origin = [0,0]):
    """
    Converts a linear index value to world coordinates (meters) in the form of a python list
    This transformation is derived from map width
    @param a linear index value, specifying a cell/pixel in an 1-D array
    @param map_width: number of columns in the occupancy grid
    @param map_resolution: side of each grid map square in meters
    @param map_origin: the x,y position in grid cell coordinates of the world's coordinate origin
    @return list containing x,y coordinates in the world frame of reference
    """
    # convert to x,y grid cell/pixel coordinates
    grid_cell_map_x = array_index % map_width
    grid_cell_map_y = array_index // map_width
    # convert to world coordinates
    x = map_resolution * grid_cell_map_x + map_origin[0]
    y = map_resolution * grid_cell_map_y + map_origin[1]

    return [x,y]

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
  # To-Do: replace numerical constants preferably with parameters / static_map srv
  # side of each grid map square in meters
  resolution = 0.2
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  origin = [-7.4, -7.4, 0]

  # time statistics
  start_time = rospy.Time.now()

  # calculate the shortes path using A-star
  path = a_star(start_index, goal_index, width, height, costmap, resolution, origin)

  if not path:
    rospy.logwarn("No path returned by A-star")
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

def a_star(start_index, goal_index, width, height, costmap, resolution, origin):
  ''' 
  Performs A-star's shortes path algorithm search on a costmap with a given start and goal node
  '''

  # create an open_list
  open_list = []

  open_list.append([start_index, 0])

  # set to hold already processed nodes
  closed_nodes = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()
  g_costs[start_index] = 0

  # dict for mapping f costs to nodes
  f_costs = dict()
  f_costs[start_index] = 0

  shortest_path = []

  path_found = False
  rospy.loginfo('A-Star: Done with initialization')

  # Main loop, executes while there are still nodes in open_list
  while open_list:
    # sort open_list according to the second element of each sublist
    open_list.sort(key = lambda x: x[1]) 
    # extract the first element (the one with the shortes travel cost)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_nodes.add(current_node)

    # If current_node is the goal, exit the search loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_nodes:
        continue

      # calculate g value of neighbour if movement passes through current_node
      g_cost = g_costs[current_node] + step_cost

      # pure heuristic 'h_cost'
      from_xy = indexToWorldCoord(current_node, width, resolution, origin)
      to_xy = indexToWorldCoord(goal_index, width, resolution, origin)
      h_cost = euclidean_distance(from_xy, to_xy)

      # A Star's heuristic value
      f_cost = g_cost + h_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if f_cost < f_costs[neighbor_index]:
          # Update the node's g_cost (travel cost)
          g_costs[neighbor_index] = g_cost
          # Update the node's f_cost (A-Star heuristic)
          f_costs[neighbor_index] = f_cost
          parents[neighbor_index] = current_node
          # Update the node's f_cost inside open_list
          open_list[idx] = [neighbor_index, f_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost (travel cost)
        g_costs[neighbor_index] = g_cost
        # Set the node's f_cost (A-Star heuristic)
        f_costs[neighbor_index] = f_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, f_cost])

  rospy.loginfo('A-Star: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('A-Star: No path found!')
    return shortest_path

  ## Build path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          node = parents[node]
          shortest_path.append(node)
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('A-Star: Done reconstructing path')

  # print statistics
  rospy.loginfo('++++++++ A-Star execution metrics ++++++++')
  rospy.loginfo('Total nodes expanded: %s', str(len(closed_nodes)))
  rospy.loginfo('Nodes in frontier: %s', str(len(open_list)))
  rospy.loginfo('Unvisited nodes (this includes obstacles): %s', str((height * width)-len(closed_nodes)-len(open_list))) # note: this number includes obstacles

  return shortest_path

# Creates service 'make_plan', service requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

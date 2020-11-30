#! /usr/bin/env python

"""
ROS service server for Dijkstra's algorithm path planning exercise
Author: Roberto Zegers R.
Date: Nov 30, 2020
Usage: roslaunch unit3_exercises_pp unit_3.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
import heapq
import math

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

def pick_current_node(queue):
   # The order that we iterate over the nodes is controlled by a priority queue
   # Pop and return the smallest item from the heap, maintaining the heap invariant
  return heapq.heappop(queue)

def update_unvisited(distance_value, n_index, unvisited):
  # Push 'entry' onto the heap, maintaining the heap invariant
  entry = [distance_value, n_index]
  heapq.heappush(unvisited, entry)

def find_neighbors(index, width, map_size, costmap):

  neighbors = []
  check = []
  # upper
  if index - width > 0:
    check.append([index - width, 1])

  # left
  if (index - 1) % width > 0:
    check.append([index - 1, 1])

  # upper left
  if index - width - 1 > 0 and (index - width - 1) % width > 0:
    check.append([index - width - 1, 1.4])

  # upper right
  if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
    check.append([index - width + 1, 1.4])

  # right
  if (index + 1) % width != (width + 1):
    check.append([index + 1, 1])

  # lower left
  if (index + width - 1) < map_size and (index + width - 1) % width != 0:
    check.append([index + width - 1, 1.4])

  # lower
  if (index + width) <= map_size:
    check.append([index + width, 1])

  # lower right
  if (index + width + 1) <= map_size and (index + width + 1) % width != (width - 1):
    check.append([index + width + 1, 1.4])


  for element in check:
    if costmap[element[0]] == 0:
      neighbors.append(element)

  return neighbors


def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a PathPlanningPluginResponse
  '''
  # This is the data you get from the request
  # The costmap is a 1-D array version of the original costmap image
  costmap = req.costmap_ros
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal

  # Calculate the shortes path using dijkstra
  path = dijkstra(start_index, goal_index, width, height, costmap)

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('Dijkstra: Path send to navigation stack')
  return resp

####### Paste code from exercise 3.5.4. HERE ################
## Build path by working backwards from target
def build_path(prev, source, target):
  path_array = []
  current_node = target
  path_array.append(current_node)

  # Continue until reaching the source node
  while True:
    parent = prev[current_node]
    if parent == source:
      break
    elif parent == float("inf"):
      break
    path_array.append(parent)
    current_node = parent

  # use extended slicing to reverse the path
  path_array = path_array[::-1]

  rospy.loginfo('Dijkstra: Done building path')
  rospy.logdebug(path_array)

  return path_array
####### End of code from exercise 3.5.4. ###############

def dijkstra(start_index, goal_index, width, height, costmap):
  ''' 
  Implementation of Dijkstra's algorithm
  '''
  map_size = height * width

  ### Paste code from exercise 3.5.1. HERE ###
  # 1. Add a list called 'distance' to keep track of the total cost from the start node to each node
  #    use infinity as a default distance for all nodes
  # 2. Set distance of start index to zero
  # 3. Add a list called 'previous' to keep track of each nodes's parent node, required to construct the path
  #    use infinity as a default parent node for all nodes
  # 4. Add an empty list called 'neighbors' to contain the indices of all grid cells reachable from current node
  # 5. Add an empty list called 'unvisited' to keep track of the nodes discovered but not yet visited
  # 6. Add the start node to the list of unvisited nodes
  #    Use the update_visited() function to keep the list sorted in an memory efficient manner

  path = []
  distance = [float("inf")] * map_size
  distance[start_index] = 0
  previous = [float("inf")] * map_size
  neighbors = []
  unvisited = []
  update_unvisited(distance[start_index], start_index, unvisited)
  ### End of code from exercise 3.5.1. ###

  rospy.loginfo('Dijkstra: Done with initialization')

  #### Paste code from exercise 3.5.2. HERE ####
  # Main loop: Continue while there are still nodes in the list of unvisited nodes
  while unvisited:
    # Get a new current_node using the provided 'pick_current_node()' function
    # If current_node is the goal, exit the search loop
    current_distance, current_vertex = pick_current_node(unvisited)
    if current_vertex == goal_index:
      break
    #### End of code from exercise 3.5.2. ###

    #### Enter code from exercise 3.5.3. HERE ###
    neighbors = find_neighbors(current_vertex, width, map_size, costmap)

    for neighbor in neighbors:
      # unpack neighbor
      neighbor_distance = neighbor[1]
      neighbor_index = neighbor[0]

      if (distance[neighbor_index] > (distance[current_vertex] + neighbor_distance)):
        # Update the smallest tentative distance for the current neighbor
        distance[neighbor_index] = distance[current_vertex] + neighbor_distance
        previous[neighbor_index] = current_vertex
        # Add the current neighbor to list of unvisited nodes using the update_visited() function
        update_unvisited(distance[neighbor_index], neighbor_index, unvisited)
    #### End of code from exercise 3.5.3. ####

  rospy.loginfo('Dijkstra: Done traversing list of unvisited nodes')
  ## Build path by working backwards from target
  path = build_path(previous, start_index, goal_index)

  return path

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

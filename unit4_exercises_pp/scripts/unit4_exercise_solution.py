#! /usr/bin/env python

"""
ROS service server for A-Star search algorithm, path planning exercise
Author: Roberto Zegers R.
Date: Nov 30, 2020
Usage: roslaunch unit4_exercises_pp unit_4.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
import heapq
from math import sqrt

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

# global variables for Rviz visualization


def pop_current_node(queue):
   # The order that we iterate over the nodes is controlled by a priority queue
   # Pop and return the smallest item from the heap, maintaining the heap invariant
  return heapq.heappop(queue)

def add_unvisited(distance_value, n_index, unvisited):
  # Push 'entry' onto the heap, maintaining the heap invariant
  # Heap element is the distance value alongside the node index being tracked
  entry = [distance_value, n_index]
  # Heaps are binary trees for which every parent node has a value less than or equal to any of its children
  heapq.heappush(unvisited, entry)

def update_rviz(index, state):
  """
  updates Rviz visulization marker
  """


def find_neighbors(index, width, map_size, costmap):
  """
  Returns a compound list containing several neighbour nodes as index/distance pairs
  """

  neighbors = []
  check = []
  # upper
  if index - width > 0:
    check.append([index - width, 1])

  # left
  if (index - 1) % width > 0:
    check.append([index - 1, 1])

  # upper left
  """
  if index - width - 1 > 0 and (index - width - 1) % width > 0:
    check.append([index - width - 1, 1.4])

  # upper right
  if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
    check.append([index - width + 1, 1.4])
  """

  # right
  if (index + 1) % width != (width + 1):
    check.append([index + 1, 1])

  # lower left
  """
  if (index + width - 1) < map_size and (index + width - 1) % width != 0:
    check.append([index + width - 1, 1.4])
  """

  # lower
  if (index + width) <= map_size:
    check.append([index + width, 1])

  # lower right
  """
  if (index + width + 1) <= map_size and (index + width + 1) % width != (width - 1):
    check.append([index + width + 1, 1.4])
  """


  for element in check:
     # Check if the node is a wall
    if costmap[element[0]] == 0:
      # appenda a pair to a list
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

  # Calculate the shortes path using pathfinder function
  path = pathfinder(start_index, goal_index, width, height, costmap)

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('A-Star: Path send to navigation stack')
  return resp

####### Paste code from exercise 3.5.4. HERE ################
## Build path by working backwards from target
def build_path(prev, source, target):
  """
  @param list containing parent nodes
  @param start node 1-D index
  @param goal node 1-D index
  @return list containing path from start to goal
  """
  path_array = []
  current_node = target
  rospy.loginfo('A-Star: Start reconstructing path')
  path_array.append(current_node)
  rospy.logdebug('A-Star: Added current node %s to path', str(current_node))


  # Continue until reaching the source node
  while current_node != source:
    parent = prev[current_node]
    if parent == source:
      break
    elif parent == float("inf"):
      break
    path_array.append(parent)
    rospy.logdebug('A-Star: Added node %s, parent of %s to path', str(parent), str(current_node))
    current_node = parent

  # use extended slicing to reverse the path
  path_array = path_array[::-1]

  rospy.loginfo('A-Star: Done reconstructing path')
  rospy.logdebug(path_array)


  return path_array

####### End of code from exercise 3.5.4. ###############

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

def heuristic(source_node, target_node):
  """
  Calculates the linear_distance between two nodes (euclidean distance).
  The linear_distance is an estimation of the real distance without taking into account obstacles
  @param from_node: the node from which to check (x,y grid cell coordinates)
  @param to_node: the node to which to check (x,y grid cell coordinates)
  @return: the distance

  dx = target_node[0] - source_node[0]
  dy = target_node[1] - source_node[1]
  linear_distance = sqrt(dx ** 2 + dy ** 2)
  return linear_distance  
  """
  DiagonalCost = 2
  OrthogonalCost = 1

  # the number of steps you can take along a diagonal
  h_diagonal = min(abs(source_node[0] - target_node[0]), abs(source_node[1] - target_node[1]))
  # = the Manhattan distance
  h_straight = (abs(source_node[0] - target_node[0]) + abs(source_node[1] - target_node[1]))

  heuristic = DiagonalCost * h_diagonal + OrthogonalCost * (h_straight - 2*h_diagonal)

  return heuristic

def assign_priority_value(current_node_index, target_node_index):
  """
  Assigns a new priority value to a node.
  A priority value defines the order by which nodes are processed.
  The node with the lowest value comes first.
  @param current_node_index: the node from which to check (1-D array index value)
  @param target_node_index: the node to which to check (1-D array index value)
  @return: new priority value
  """
  return calculateLinearDistance(current_node_index, target_node_index)

def pathfinder(start_index, goal_index, width, height, costmap):
  ''' 
  Implementation of A-Star's algorithm
  '''
  map_size = height * width

  ### Paste code from exercise 3.5.1. HERE ###
  # 1. Add a list called 'priority_value' to keep track of the total cost from the start node to each node
  #    use infinity as a default priority_value for all nodes
  # 2. Set priority_value of start index to zero
  # 3. Add a list called 'previous' to keep track of each nodes's parent node, required to construct the path
  #    use infinity as a default parent node for all nodes
  # 4. Add an empty list called 'neighbors' to contain the indices of all grid cells reachable from current node
  # 5. Add an empty list called 'unvisited' to keep track of the nodes discovered but not yet visited
  # 6. Add the start node to the list of unvisited nodes
  #    Use the update_visited() function to keep the list sorted in an memory efficient manner

  path = []
  # to keep shortest distance, used as priority value
  # all nodes receive a score of INF
  priority_value = [float("inf")] * map_size

  ### this shold be different in greedy search???  It should not have priority_value ???? what about the 'parent node' in greedy search

  # score of the start node is set to 0
  from_node = indexToGridCell(start_index, width)
  to_node = indexToGridCell(goal_index, width)
  priority_value[start_index] = heuristic(from_node, to_node)
  # predecessors
  previous = [float("inf")] * map_size
  neighbors = []
  # Create list holds the nodes to process (as heap)
  unvisited = []
  # Create list holds the nodes to process
  closed_nodes = []
  # Add the origin to the unvisited queue
  add_unvisited(priority_value[start_index], start_index, unvisited)
  ### End of code from exercise 3.5.1. ###
  count = 0
  rospy.loginfo('A-Star: Done with initialization')
  

  #### Paste code from exercise 3.5.2. HERE ####
  # Main loop: Continue while there are still nodes in the list of unvisited nodes
  while unvisited:
    rospy.logdebug('A-Star: List of unvisited nodes %s:', str([item[1] for item in unvisited]))
    # Pop the node with the lowest score using the provided 'pop_current_node()' function
    current_distance, current_vertex = pop_current_node(unvisited)
    #rospy.loginfo('A-Star: selected new node: %s', str(current_vertex))
    count += 1

    # Exit the search loop early if current_node is the goal
    if current_vertex == goal_index:
      break
    #### End of code from exercise 3.5.2. ###

    #### Enter code from exercise 3.5.3. HERE ###
    # Get neighbors
    neighbors = find_neighbors(current_vertex, width, map_size, costmap)
    #rospy.loginfo('A-Star: found these neighbors: %s', str(neighbors))

    # Loop neighbors
    for neighbor in neighbors:
      # unpack neighbor into index and distance
      neighbor_index = neighbor[0]
      distance_score_to_neighbor = neighbor[1]

      # Check if the neighbor has already been visited (visited nodes, this means that those are nodes that were already "searched")
      # # Either update previous cell or add new cell to the frontier ??
      if neighbor_index not in closed_nodes:
        #rospy.loginfo('A-Star: is processing neighbor: %s', str(neighbor_index))
        ### A-Star
        # Calculate new cost of going to a certain neighbor in a given direction
        cost_from_start = priority_value[current_vertex] + distance_score_to_neighbor
        # priority = cost_from_start + cost_to_target

        ### Greedy Best-First-Search
        # convert to x,y coordinates
        from_node = indexToGridCell(current_vertex, width)
        to_node = indexToGridCell(goal_index, width)

        cost_to_target = heuristic(from_node, to_node)

        ###### A Star
        #cost_from_start = priority_value[current_vertex] + distance_score_to_neighbor
        #cost_to_target = heuristic(goal, next)
        # priority = cost_from_start + cost_to_target

        # If new distance is lower than neighbor's current distance (includes never visited nodes since they have a distance 'inf')
        #if ((priority_value[current_vertex] + distance_score_to_neighbor) < priority_value[neighbor_index]):
        if (cost_to_target < priority_value[neighbor_index]):
          # 1. Update the smallest tentative distance_score for the current neighbor
          priority_value[neighbor_index] = cost_to_target + cost_from_start
          # 2. Keep track of parent node's index
          previous[neighbor_index] = current_vertex
          #rospy.loginfo('A-Star: set as parent of node %s the following: %s', str(neighbor_index), str(current_vertex))
          # 3. Add the current neighbor to list of unvisited nodes using the update_visited() function
          if not neighbor_index in [x[1] for x in unvisited]:
            rospy.logdebug('Adding neighbor_index twice into heap')  
            add_unvisited(priority_value[neighbor_index], neighbor_index, unvisited)
          ### print unvisited maybe we are adding it twice


    # Close current_vertex since all of its neighbours have been added to the open list (if necessary)
    # To prevent from visting the node again
    closed_nodes.append(current_vertex)

    #### End of code from exercise 3.5.3. ####

  rospy.loginfo('A-Star: Done traversing list of unvisited nodes')
  rospy.loginfo('A-Star: Traversed %d nodes', count)
  ## Reconstruct shortest path by working backwards from target
  path = build_path(previous, start_index, goal_index)

  """
  rospy.loginfo('A-Star: path found: %s', str(path))
  for cell in path:
    rospy.loginfo('A-Star: distance for cell %s is: %s', str(cell), str(priority_value[cell]))
  """
  
  return path

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

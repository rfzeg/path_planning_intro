#! /usr/bin/env python

"""
ROS Artificial Potential Fields gradient descent path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: May 11, 2021
Usage: roslaunch unit5_pp unit5_exercise.launch
"""

import rospy

def lowest_neighbor(index, width, height, potential_field):
    """
    Identifies neighbor node located towards the direction of steepest descent
    Inspects the 8 adjacent neighbors
    Checks if neighbor is inside the map boundaries
    Returns index of neighbor node in the direction of steepest descent
    """
    candidates = []

    # Get the index value of the 8 adjacent neighbors
    upper = index - width
    left = index - 1
    upper_left = index - width - 1
    upper_right = index - width + 1
    right = index + 1
    lower_left = index + width - 1
    lower = index + width
    lower_right = index + width + 1

    # upper neighbor
    if upper > 0:
      # discrete_gradient = potential force at current cell - potential force at candidate cell
      discrete_gradient = potential_field[upper] - potential_field[lower]
      candidates.append([discrete_gradient, upper])

    # left neighbor
    if left % width > 0:
      discrete_gradient = potential_field[left] - potential_field[right]
      candidates.append([discrete_gradient, left])

    # upper left neighbor
    if upper_left > 0 and upper_left % width > 0:
      discrete_gradient = potential_field[upper_left] - potential_field[lower_right]
      candidates.append([discrete_gradient, upper_left])

    # upper right neighbor
    if upper_right > 0 and (upper_right) % width != (width - 1):
      discrete_gradient = potential_field[upper_right] - potential_field[lower_left]
      candidates.append([discrete_gradient, upper_right])

    # right neighbor
    if right % width != (width + 1):
      discrete_gradient = potential_field[right] - potential_field[left]
      candidates.append([discrete_gradient, right])

    # lower left neighbor
    if lower_left < height * width and lower_left % width != 0:
      discrete_gradient = potential_field[lower_left] - potential_field[upper_right]
      candidates.append([discrete_gradient, lower_left])

    # lower neighbor
    if lower <= height * width:
      discrete_gradient = potential_field[lower] - potential_field[upper]
      candidates.append([discrete_gradient, lower])

    # lower right neighbor
    if (lower_right) <= height * width and lower_right % width != (width - 1):
      discrete_gradient = potential_field[lower_right] - potential_field[upper_left]
      candidates.append([discrete_gradient, lower_right])

    return min(candidates)[1]

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
  Performs gradient descent on an artificial potential field with a given start and goal node
  '''
  # Iterations limit
  max_iterations = 1000
  # Iteration counter
  current_iteration = 0
  # Tolerance margin allowed around goal position (in grid cells)
  goal_tolerance = 3
  # Boolean flag indicating whether the goal was reached or not
  path_found = False
  # Index corresponding to the last grid cell added to the path
  current = start_index
  # A list to keep all path waypoints
  path = []

  rospy.loginfo('Gradient descent: Done with initialization')

  # Loop that iterates until the maximum allowed number of loops is met
  while (current_iteration < max_iterations):

    # Optional: Visualize gradient descent path in Rviz
    descent_viz.draw(current, potential_field_data[current])

    # Check if goal was reached using tolerance value
    if (goal_tolerance > euclidean_distance(current, goal_index, width)):
      path_found = True
      rospy.loginfo('Gradient descent: Goal reached')
      # Optional: Visualize gradient descent path in Rviz
      descent_viz.draw(goal_index, 0)
      break

    # Neighbor cell with the lowest potential force value
    new_waypoint = lowest_neighbor(current, width, height, potential_field_data)
    # Add it to the path
    path.append(new_waypoint)
    # Set new waypoint as the new grid cell located at the tip of the path (for the next algorithm iteration)
    current = new_waypoint
    # Increase the number of iteration by one
    current_iteration += 1
  
  # Check for local minima
  path_set = set(path)
  if len(path) != len(path_set):
    rospy.logwarn('Gradient descent probably stuck at local minima!!')

  if not path_found:
    rospy.logwarn('Gradient descent path could not reach the goal!')
    return
  else:
    return path

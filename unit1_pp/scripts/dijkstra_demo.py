#! /usr/bin/env python

"""
ROS service for Dijkstra's shortest path algorithm demo
Author: Roberto Zegers R.
Date: Feb 22, 2021
Usage: roslaunch unit1_pp dijkstra_demo.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from openlist import OpenList

def find_neighbors(index, width, height, costmap, orthogonal_movement_cost):

  neighbors = []
  check = []
  diagonal_movement_cost = orthogonal_movement_cost * 1.41421

  if index - width > 0:
    check.append([index - width, orthogonal_movement_cost])
  if (index - 1) % width > 0:
    check.append([index - 1, orthogonal_movement_cost])
  if index - width - 1 > 0 and (index - width - 1) % width > 0:
    check.append([index - width - 1, diagonal_movement_cost])
  if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
    check.append([index - width + 1, diagonal_movement_cost])
  if (index + 1) % width != (width + 1):
    check.append([index + 1, orthogonal_movement_cost])
  if (index + width - 1) < height * width and (index + width - 1) % width != 0:
    check.append([index + width - 1, diagonal_movement_cost])
  if (index + width) <= height * width:
    check.append([index + width, orthogonal_movement_cost])
  if (index + width + 1) <= height * width and (index + width + 1) % width != (width - 1):
    check.append([index + width + 1, diagonal_movement_cost])

  for element in check:
    if costmap[element[0]] == 0:
      neighbors.append(element)

  return neighbors


def make_plan(req):

  costmap = req.costmap_ros
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  resolution = 0.1
  origin = [-20.0, -20.0, 0.0]
  start_time = rospy.Time.now()

  path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin)

  if not path:
    rospy.logwarn("No path returned by Dijkstra")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Dijkstra execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Dijkstra: Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('Dijkstra: Response sent to navigation stack')
  return resp

def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin):

  open_list = OpenList()
  open_list.put(start_index, 0)
  closed_nodes = set()
  parents = dict()
  path_found = False
  costs = dict()
  costs[start_index] = 0

  path = []

  rospy.loginfo('Dijkstra: Done with initialization')

  while not open_list.empty():

    current_node = open_list.get()
    closed_nodes.add(current_node)

    if current_node == goal_index:
      path_found = True
      break

    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    for neighbor_index, step_cost in neighbors:

      if neighbor_index in closed_nodes:
        continue

      g_cost  = costs[current_node] + step_cost

      if open_list.contains(neighbor_index):
        if g_cost < open_list.get_priority(neighbor_index):
          costs[neighbor_index] = g_cost
          parents[neighbor_index] = current_node
          open_list.decrease_key(neighbor_index, g_cost)
      else:
        costs[neighbor_index] = g_cost
        parents[neighbor_index] = current_node
        open_list.put(neighbor_index, g_cost)

  rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('Dijkstra: No path found!')
    return path

  if path_found:
      node = goal_index
      path.append(goal_index)
      while node != start_index:
          node = parents[node]
          path.append(node)

  path = path[::-1]

  return path

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('dijkstra_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)

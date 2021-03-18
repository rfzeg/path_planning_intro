#!/usr/bin/env python

"""
ROS service server node for A-Star path planning algorithm exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: March 2021
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from unit3_astar_exercise import a_star

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
  # origin of grid map
  origin = [-7.4, -7.4, 0]

  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

  # time statistics
  start_time = rospy.Time.now()

  # calculate the shortes path using A-star
  path = a_star(start_index, goal_index, width, height, costmap, resolution, origin, viz)

  if not path:
    rospy.logwarn("No path returned by A-star")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('+++++++++ A-Star execution metrics +++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('A-Star: Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('astar_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
#! /usr/bin/env python

"""
ROS service server that takes in path planning requests and
responds with a plan for global path planning.
Author: Roberto Zegers R.
Usage: rosrun [package_name] unit3_exercise.py
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

def make_plan(req):
  ''' 
  Callback function used to process incoming requests
  It returns a PathPlanningPluginResponse
  '''
  # decompose incoming request msg into individual elements
  costmap = req.costmap_ros   # 1-D array version of the original costmap image
  width = req.width
  height = req.height
  map_size = height * width
  start_index = req.start     # start position
  goal_index = req.goal       # desired goal position
  
  # Minimal implementation: a straight line from the start position to the desired goal position
  path = []
  path.append(start_index)
  path.append(goal_index)

  rospy.loginfo('Path found:')
  rospy.loginfo(path)

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

# create a service, specify its name, type and callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

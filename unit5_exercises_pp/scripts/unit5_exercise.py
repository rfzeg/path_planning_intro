#! /usr/bin/env python

"""
ROS service server for Rapidly exploring random trees (RRT) algorithm path planning exercise
Author: Roberto Zegers R.
Date: December 2020
Usage: roslaunch unit5_exercises_pp unit_5.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse

rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)

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

  # Calculate a path using RRT
  path = rrt(start_index, goal_index, width, height, costmap)

  # make a response object
  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('RRT: Path send to navigation stack')
  return resp

def rrt(start_index, goal_index, width, height, costmap):
  ''' 
  Implementation of Rapidly exploring random trees (RRT) algorithm
  '''
  map_size = height * width

  # Enter the code from the exercises below
  # At the moment the path contains only the start and goal positions
  path = []
  # add start location to plan
  path.append(start_index)
  # insert goal location to plan
  path.append(goal_index)

  return path

# create a service named 'make_plan', requests are passed to the make_plan callback function
make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)

rospy.spin()

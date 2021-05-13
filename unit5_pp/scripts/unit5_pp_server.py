#!/usr/bin/env python

"""
ROS service server node for artificial potential fields exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: May 2021
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalID
from descentviz import DescentViz
from gradient_descent import gradient_descent

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  '''
  global got_potential_field_update
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

  # wait for total potential field update
  got_potential_field_update = False
  while not got_potential_field_update:
    pass

  viz = DescentViz(start_index, width, resolution, origin, id=1, frame="map")

  # time statistics
  start_time = rospy.Time.now()

  # calculate path using gradient descent
  path = gradient_descent(start_index, goal_index, width, height, total_potential_field, viz)

  if not path:
    rospy.logwarn("Path server: No path returned by gradient descent")
    path = []
    # cancel move_base's navigation goal
    cancel_goal_pub.publish(GoalID())
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('+++++++++++ Gradient Descent execution metrics ++++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Path server: Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

def total_potential_field_callback(msg):
    global total_potential_field, got_potential_field_update
    # total potential field as 1-D array representation
    total_potential_field = msg.data
    got_potential_field_update = True

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('gradient_descent_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  total_field_subscriber = rospy.Subscriber("total_field", OccupancyGrid, total_potential_field_callback)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
  total_potential_field  = None
  got_potential_field_update = False
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
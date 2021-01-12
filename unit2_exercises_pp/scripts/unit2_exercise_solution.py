#! /usr/bin/env python

"""
ROS node Occupancy Grid Mapping algorithm exercise
Author: Roberto Zegers R.
Date: Jan 12, 2021
Usage: roslaunch unit2_exercises_pp unit_2.launch
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point, Pose, TransformStamped
from tf.transformations import euler_from_quaternion

from math import ceil, isinf, sin, cos, log
import bresenham

# global variable to keep x,y,yaw values in the form of a python list
robot_pose=[0.0, 0.0, 0.0]

class localmap:
    def __init__(self, height, width, resolution,morigin):
        self.height=height
        self.width=width
        self.resolution=resolution
        self.p_unknown=-1.0
        # binary occupancy grid (1-D array localmap, initilized as unknown)
        self.localmap=[self.p_unknown]*int(self.width/self.resolution)*int(self.height/self.resolution)

        # probabilistic occupancy grid (1-D logodds array to keep track of probablilites of each cell)
        # This term represents the prior probability, in log odds form, of the occupancy state of each cell
        self.logodds=[0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        # index of map origin cell?
        self.origin=int(ceil(morigin[0]/resolution))+int(ceil(width/resolution)*ceil(morigin[1]/resolution))

        # Log-Probabilities used to update the log odds map (can be tuned)
        # log odds measurement model parameter for free cells
        self.pfree=log(0.3/0.7) # P(A)/P(not A)
        # log odds measurement model parameter for occupied cells
        self.pocc=log(0.9/0.1) # P()/P(not )
        # at map initialization
        self.prior=log(0.5/0.5)
        # clip or clamp logodd value
        self.max_logodd=100.0
        # used to transfer probabilistic occupancy grid into a binary occupancy grid
        self.max_logodd_belief=10.0

        self.map_origin=morigin


    def updatemap(self,scandata,angle_min,angle_max, angle_increment,range_min,range_max,robot_pose):
        #The robot position as an index in a 1-D array map
        # robot_origin=int(robot_pose[0])+int(ceil(self.width/self.resolution)*robot_pose[1])

        # get the ray in the center
        # devide len(scandata) with 2 and plus 1.
        # it mean the center of the ray.

        # iterate over all laser rays
        for i in range(len(scandata)):
            # and we can skip 'inf' data with following code.
            # isinf: if there is 'inf' data, return true.

            # min is required to filter out 'inf' values, in that case 5 is used (max laser scanner range)
            dist_value = min(scandata[i], 5)

            # If laser ray is not 'Inf'
            inf_measurement = False
            if isinf(scandata[i]):
                inf_measurement = True
                
            # angle_increment : angular distance between two nearby rays
            # ray_angle : angular distance 
            ray_angle = angle_min + (i * angle_increment)

            # rospy.logdebug("Ray nr. %d,  angle (deg): %f, range: %f " % (i, ray_angle*57.3, scandata[i]))
            # rospy.logdebug("Scanner bearings:")
            # rospy.logdebug("Scanner ranges:")

            # Find position of the detected obstacle in the global frame, this is simple geometry
            # cos(robot_pose[2]+ray_angle) : the angle of laser ray
            # float(dist_value) : the distance measured by the ray
            # resolution : the map resolution    
            px = int(robot_pose[0]/self.resolution) + int(float(dist_value)*cos(robot_pose[2]+ray_angle)/self.resolution)
            py = int(robot_pose[1]/self.resolution) + int(float(dist_value)*sin(robot_pose[2]+ray_angle)/self.resolution)

            # rospy.logdebug("Ray nr. %d,  map pixel (x,y): %d,%d " % (i, px, py))

            # initialize 'line' object, consider current robot robot_pose and px,py of laser measurement
            line = bresenham.bresenham([int(robot_pose[0]/self.resolution),int(robot_pose[1]/self.resolution)],[px,py])

            # iterate over all pixels in 'line' object
            for j in range(len(line.path)):
                # lpx, lpy are all the pixel from start point to end point
                lpx=self.map_origin[0]+robot_pose[0]+line.path[j][0]*self.resolution
                lpy=self.map_origin[1]+robot_pose[1]+line.path[j][1]*self.resolution

                # check that lpx and lpy are inside of the map boundary
                if (0<=lpx<self.width and 0<=lpy<self.height):
                    # get 1-D array index
                    index=self.origin+int(line.path[j][0]+ceil(self.width/self.resolution)*line.path[j][1])
                    # verify that index is inside of 1-D array map
                    if (index < len(self.logodds)):

                        # if laser measurement is within sensor boundary
                        if dist_value < range_max:
                            # if it is NOT the last pixel in the 'line' object
                            if(j<len(line.path)-1):
                              # mark pixel as free: idea use updateFreeCell() function
                              #  for a cell measured as being free we update the logodds
                              #  by (subtracting?) the logodds-free measurement parameter self.pfree
                              self.logodds[index]+=self.pfree
                            # if it is the last pixel in the 'line' object
                            else:
                              # mark pixel as occupied: idea use updateOccupiedCell() function
                              #  for a cell measured as being occupied we update the logodds
                              #  by adding the logout occupied measurement parameter self.pocc
                              self.logodds[index]+=self.pocc

                        # if laser measurement is out of sensor boundary
                        else:
                          # mark pixel as free
                          self.logodds[index]+=self.pfree

                        # clip: check if loggods is above max logodds value
                        # saturation, threshold the cells value for numerical stability
                        if self.logodds[index]>self.max_logodd:
                            self.logodds[index]=self.max_logodd

                        # clip: check if loggods is below min logodds value
                        # saturation, threshold the cells value for numerical stability
                        elif self.logodds[index]<-self.max_logodd:
                            self.logodds[index]=-self.max_logodd

                        # transfer probabilistic occupancy grid into a binary occupancy grid
                        # The maximum likelihood map is obtained by clipping the occupancy grid map at a
                        # threshold of 0.5 (???)
                        # After threshold, all values are either (0 or 100)
                        if self.logodds[index]>self.max_logodd_belief:
                            self.localmap[index]=100
                        else:
                            self.localmap[index]=0

                        # set value of map's origin grid cell to  (100)  ??
                        self.localmap[self.origin]=100.0
                else:
                  rospy.logwarn("lpx and lpy are out of boundary, skipping beam")


def odometryCb(msg):
    global robot_pose

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    # transforming quaternions to euler 
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # pack the robot's position and orientation in the map frame of reference as a python list
    robot_pose=[x,y,yaw]
    #rospy.logdebug("Robot pose in map frame (x, y, yaw(rad)): %s", str(robot_pose))

def scanCb(msg):

    scandata=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    range_min=msg.range_min
    range_max=msg.range_max

    # Update map
    m.updatemap(scandata,angle_min,angle_max,angle_increment,range_min,range_max,robot_pose)

def mappublisher(m,height,width,resolution,origin):
    msg=OccupancyGrid()
    msg.header.frame_id='map'
    msg.info.resolution=resolution
    msg.info.width=ceil(width/resolution)
    msg.info.height=ceil(height/resolution)
    msg.info.origin.position.x=-origin[0]
    msg.info.origin.position.y=-origin[1]
    msg.data=m
    mappub.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node('mapping_exercise', log_level=rospy.DEBUG)
        rospy.Subscriber('/odom', Odometry, odometryCb)
        rospy.Subscriber('/scan', LaserScan, scanCb)
        mappub=rospy.Publisher('/map',OccupancyGrid,queue_size=1)

        rate = rospy.Rate(20)

        # parameters
        height,width,resolution=20,20,0.1
        map_origin=[width/2.0, height/2.0]

        # initialize instance of localmap
        m=localmap(height, width, resolution, map_origin)

        # clean shutdown rospy sleep timer
        while not rospy.core.is_shutdown():
            mappublisher(m.localmap, height, width, resolution, map_origin)
            rospy.rostime.wallsleep(0.5)
        rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)


    except rospy.ROSInterruptException:
        pass


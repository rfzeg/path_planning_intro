# Notice: This file has been adapted from the original file 
# 'planner.py' from the route_network ROS package code, found at:
# https://github.com/ros-geographic-info/open_street_map/blob/46b699beb8f29c1bdfb3ea088e94b9b96c93c3b9/route_network/src/route_network/planner.py
#
# It was adapted for illustrative purposes only and should not
# be considered a replacement for the original file. This is a
# trimmed-down version that lacks part of the original functionallity
# All credit goes to the authors of the original planner.py file

##################################################################

# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Route network path planner.

.. _`geographic_msgs/GetRoutePlan`: http://ros.org/doc/api/geographic_msgs/html/srv/GetRoutePlan.html
.. _`geographic_msgs/RouteNetwork`: http://ros.org/doc/api/geographic_msgs/html/msg/RouteNetwork.html
.. _`geographic_msgs/RoutePath`: http://ros.org/doc/api/geographic_msgs/html/msg/RoutePath.html
.. _`uuid_msgs/UniqueID`: http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html

"""

import numpy
import math
import geodesy.utm
import geodesy.wu_point
import rospy

from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath
from geographic_msgs.msg import RouteSegment
from geographic_msgs.srv import GetRoutePlan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class PlannerError(Exception):
    """Base class for exceptions in this module."""

class NoPathToGoalError(PlannerError):
    """Exception raised when there is no path to the goal."""

class Edge():
    """
    :class:`Edge` stores graph edge data for a way point.

    :param end: Index of ending way point.
    :param seg: `uuid_msgs/UniqueID`_ of corresponding RouteSegment.
    :param step_cost: Travel distance from the edge's start to end.
    """
    def __init__(self, end, seg, step_cost=0.0):
        """Constructor. """
        self.end = end
        self.seg = seg
        self.step_cost = step_cost

    def __str__(self):
        return str(self.end)+' '+str(self.seg.uuid)+' ('+str(self.step_cost)+')'

class Planner():
    """
    :class:`Planner` plans a route through a RouteNetwork.
    :param graph: `geographic_msgs/RouteNetwork`_ message.
    """
    def __init__(self, graph):
        """Constructor.

        Collects relevant information from route network message,
        providing convenient access to the data.
        """
        self._shift_to_route_within = rospy.get_param('~shift_to_route_within', 7.)
        rospy.loginfo("~shift_to_route_within: %.2f", self._shift_to_route_within)

        self.graph = graph
        self.points = geodesy.wu_point.WuPointSet(graph.points)

        # Create empty list of graph edges leaving each map point
        self.edges = [[] for wp_id in xrange(len(self.points))]
        for seg in self.graph.segments:
            index = self.points.index(seg.start.uuid)
            if index is not None:
                n = self.points.index(seg.end.uuid)
                if n is not None:
                    # use 2D Euclidean distance for the step cost
                    dist = self.points.distance2D(index, n)
                    self.edges[index].append(Edge(n, seg.id,step_cost=dist))

        # Create a list with utm point for each point
        self.utm_points = dict()
        for p in self.points:
            self.utm_points[self.points.index(p.uuid())] = geodesy.utm.fromMsg(p.position())

    def __str__(self):
        val = '\n'
        for i in xrange(len(self.edges)):
            val += str(i) + ':\n'
            for k in self.edges[i]:
                val += '    ' + str(k) + '\n'
        return val

    def geo_path(self, req):
        """ Plan the shortest path between a start and a destination geopoint.

        The 'geo_path' method can receive GeoPoints out of the graph, upon such a case, the nearest segments on the OSM map are detected,
        and the planning is carried out.
        
        :pram req: The request message.
        :param req: geographic_msgs/GetGeoPath

        :return: The computed path, as well as the ids of the RouteNetwork and the start and end segments, plus the length of the path. The length is set to -1 in case of failure.
        :rtype: (geographic_msgs/GeoPoint[], uuid_msgs/UniqueID, uuid_msgs/UniqueID, uuid_msgs/UniqueID, length) 
        :raises: :exc:`ValueError` if invalid request.
        """
        # check for possible errors in request
        if math.isnan(req.start.latitude) or math.isnan(req.start.longitude):
            raise ValueError('NaN in starting point: ' + str(req.start))
        if math.isnan(req.goal.latitude) or math.isnan(req.goal.longitude):
            raise ValueError('NaN in starting point: ' + str(req.start))
        if math.isnan(req.start.altitude):
            req.start.altitude = 0.0
        if math.isnan(req.goal.altitude):
            req.goal.altitude = 0.0
        # get the nearest segment to the start point
        (start_seg, start_dist, start_lot) = self.getNearestSegment(req.start)
        if start_seg is None:
            raise ValueError('no nearest segment found for starting way point: ' + str(req.start))
        # get the nearest segment to the destination point
        (goal_seg, goal_dist, goal_lot) = self.getNearestSegment(req.goal)
        if goal_seg is None:
            raise ValueError('no nearest segment found for goal way point: ' + str(req.goal))

        # determine the path
        result = []
        try:
            route_path = self.dijkstra_geo_planner(start_seg, goal_seg)

        except NoPathToGoalError as e:
            return result, self.graph.id, start_seg.id, goal_seg.id, -1

        start_utm = geodesy.utm.fromMsg(req.start)
        goal_utm = geodesy.utm.fromMsg(req.goal)
        # copy the route to the result list and adds the start and/or goal position to the path
        if route_path.segments:
            seg = self._getSegment(route_path.segments[0])
            p = self._get_min_point(start_seg, start_lot)
            # add the start point if it is not close to the route
            dist_first_to_start = self.distance2D(p, start_utm)
            if dist_first_to_start > self._shift_to_route_within:
                result.append(req.start)
            result.append(p.toMsg())
            # add only the endpoints of the segments
            for index in xrange(len(route_path.segments)):
                seg_id = route_path.segments[index]
                seg = self._getSegment(seg_id)

                if index == 0:
                    # add the segment start point, if the plan segment and nearest segment are not equal
                    if not ((seg.end.uuid == start_seg.start.uuid and seg.start.uuid == start_seg.end.uuid) or (seg.start.uuid == start_seg.start.uuid and seg.end.uuid == start_seg.end.uuid)):
                        result.append(self.points[seg.start.uuid].position())
                if index+1 == len(route_path.segments):
                    # add the end point of the last segment, if the plan segment and nearest segment are not equal
                    if not ((seg.end.uuid == goal_seg.start.uuid and seg.start.uuid == goal_seg.end.uuid) or (seg.start.uuid == goal_seg.start.uuid and seg.end.uuid == goal_seg.end.uuid)):
                        result.append(self.points[seg.end.uuid].position())
                else:
                    result.append(self.points[seg.end.uuid].position())
            # add a perpendicular point or the nearest endpoint of the end segment
            p = self._get_min_point(goal_seg, goal_lot)
            result.append(p.toMsg())
            # add the destination point if it is not close to the route
            dist_last_to_goal = self.distance2D(p, goal_utm)
            if dist_last_to_goal > self._shift_to_route_within:
                result.append(req.goal)
        else:
            # direct connection
            result.append(req.start)
            result.append(req.goal)

        # calculate the distance
        last_utm = None
        dist = 0
        for point in result:
            if last_utm is None:
                last_utm = geodesy.utm.fromMsg(point)
            else:
                new_utm = geodesy.utm.fromMsg(point)
                dist += self.distance2D(last_utm, new_utm)
                last_utm = new_utm
        return result, self.graph.id, start_seg.id, goal_seg.id, dist

    def dijkstra_geo_planner(self, start_segment, goal_segment):
        """ 
        Plan route from start to goal. The actual search algorithm to find a path is executed here.   
        start_segment: The nearest segment to the start point, as type 'geographic_msgs/RouteSegment'
        goal_segment: The nearest segment to the goal point, as type 'geographic_msgs/RouteSegment'
        return: The planned path between start and goal, as type 'geographic_msgs/RoutePath'
        """

        # request parameters
        start_idx = self.points.index(start_segment.start.uuid)
        goal_idx = self.points.index(goal_segment.start.uuid)

        # initialize the object that holds the path to the goal
        plan = RoutePath(network=self.graph.id)

        # Dijkstra shortest path algorithm

        # create an open list
        # Important:
        # add nodes to open_list as [distance, index] pairs
        # where the first element is travel costs to the node
        # and the second element is the node's index
        open_list = []

        # add start node to open list as an [distance, index] pair
        open_list.append([0, start_idx])

        # boolean list used to check for closed nodes
        closed_list = [False for wp_id in xrange(len(self.points))]
        closed_list[start_idx] = True

        # dict for mapping children to parent nodes
        # Important:
        # dictionary key is current_neighbour_node
        # first element in value is current_node
        # second element is valie is the edge connecting both nodes
        parents = dict()

        # boolean flag indicating whether the goal was reached or not
        reached_goal = False

        rospy.loginfo('Dijkstra: Done with initialization')

         ##### YOUR CODE STARTS HERE #####
        


         ##### YOUR CODE ENDS HERE #####

        rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

        if not reached_goal:
            rospy.logwarn('Dijkstra: No path found!')
            plan.segments = []
            return plan

        else:
          # reconstruct path by working backwards from target
          plan.segments = []

          while current_node != start_idx:
              plan.segments.append(parents[current_node][1].seg)
              # get next node to be processed
              current_node = parents[current_node][0]

          # reverse list
          plan.segments.reverse()

          rospy.loginfo('Dijkstra: Done reconstructing path')

          return plan

    def _get_min_point(self, seg, lot):
        """ Chooses between the orthogonal projection, and the start and end points
            of the segment.

        If the given orthogonal projection lies out of the segment, the whether the start
        or end point of the segment must be chosen as the minimum point.

        :param seg: The segment.
        :type seg: geographic_msgs/RouteSegment
        :param lot: The perpendicular point to the segment.
        :type lot: geodesy.utm.UTMPoint

        :return: The perpendicular point if it is on the segment, else the start or end
                 point of the segment.
        :rtype: geodesy.utm.UTMPoint
        """
        utm_seg_start = self.utm_points[self.points.index(seg.start.uuid)]
        utm_seg_end = self.utm_points[self.points.index(seg.end.uuid)]
        length_seg = self.distance2D(utm_seg_start, utm_seg_end)
        length_lot2start = self.distance2D(lot, utm_seg_start)
        length_lot2end = self.distance2D(lot, utm_seg_end)
        if (length_lot2start <= length_seg and length_lot2end <= length_seg):
            return lot
        elif length_lot2start < length_lot2end:
            return utm_seg_start
        else:
            return utm_seg_end

    def _getSegment(self, uuid):
        """ Get the segment that corresponds to the given ID.

        :param uuid: The id of the segment.
        :type uuid: uuid_msgs/UniqueID

        :return: The segment for the given uuid.
        :rtype: geographic_msgs/RouteSegment if the segment is found, None otherwise.
        """
        for seg in self.graph.segments:
            if seg.id == uuid:
                return seg
        return None

    def _getSegmentLength(self, start_point_id, seg_id):
        """ Searches the segment with given id with given start point in a pre-cached list
            and return its length.

        :param start_point_id: The id of start point of the segment.
        :type start_point_id: uuid_msgs/UniqueID
        :param seg_id: The id of the segment.
        :type seg_id: uuid_msgs/UniqueID

        :return: Length of a segment.
        :rtype: float if the segment is found, None otherwise.
        """
        edges = self.edges[self.points.index(start_point_id.uuid)]
        for edge in edges:
            if edge.seg == seg_id:
                return edge.step_cost
        return None

    def getNearestSegment(self, geo_point, max_dist=500.):
        """ Determine the nearest segment to the given point.

        :param geo_point: The position.
        :type geo_point:  geographic_msgs/GeoPoint
        :param max_dist: The maximal allowed distance to segment.
        :type max_dist: float

        :return: A tuple of the nearest segment, which has the minimum distance to
                 given point, the distance to the segment and the perpendicular point.
        :rtype: (geographic_msgs/RouteSegment, float, geodesy.utm.UTMPoint) or
                (None, None, None), if the distance of given point to start or
                end of the segment is greater then max_dist
        """
        utm_point = geodesy.utm.fromMsg(geo_point)
        min_dist = 999999999
        result = (None, None, None)
        for seg in self.graph.segments:
            index = self.points.index(seg.start.uuid)
            if index is not None:
                n = self.points.index(seg.end.uuid)
                if n is not None:
                    # determine the length of the segment
                    length_seg = self._getSegmentLength(seg.start, seg.id)
                    if not length_seg is None:
                      length_2start = self.distance2D(utm_point, self.utm_points[index])
                      length_2end = self.distance2D(utm_point, self.utm_points[n])
                      if length_2start < max_dist or length_2end < max_dist:
                          lot_p = self.getPerpendicularPoint2D(self.utm_points[index], self.utm_points[n], utm_point)
                          length_lot2p = self.distance2D(utm_point, lot_p)
                          length_lot2start = self.distance2D(lot_p, self.utm_points[index])
                          length_lot2end = self.distance2D(lot_p, self.utm_points[n])
                          if length_lot2start <= length_seg and length_lot2end <= length_seg:
                              cur_min = length_lot2p
                              if cur_min < min_dist:
                                  min_dist = cur_min
                                  result = (seg, min_dist, lot_p)
                          else:
                              cur_min = min(length_2start, length_2end)-1.0
                              if cur_min < min_dist:
                                  min_dist = cur_min
                                  result = (seg, min_dist, lot_p)
        return result

    def getPerpendicularPoint2D(self, utm_start, utm_end, utm_p):
        """ Returns the orthongal projection of point utm_p onto a line segment (utm_start -> utm_end)

        :param utm_start: The starting point of the line segment.
        :type utm_start: geodesy.utm.UTMPoint
        :param utm_end: The ending point of the line segment.
        :type utm_end: geodesy.utm.UTMPoint
        :param utm_p: The point.
        :type utm_p: geodesy.utm.UTMPoint

        :return: The orthogonal projection (cut point) if no errors, None otherwise.
        :rtype: geodesy.utm.UTMPoint
        """
        s = numpy.array([utm_start.easting, utm_start.northing])
        e = numpy.array([utm_end.easting, utm_end.northing])
        p = numpy.array([utm_p.easting, utm_p.northing])
        rv = e - s #direction vector of the line
        rv_2 = (rv*rv).sum()
        if rv_2 == 0.:
            raise ValueError('invalid segment length')
        try:
            lamda = ((p*rv).sum() - (s*rv).sum()) / rv_2
            lot_p = s + lamda * rv
        except:
            import traceback
            print traceback.format_exc()
        return geodesy.utm.UTMPoint(lot_p[0], lot_p[1], zone=utm_p.gridZone()[0], band=utm_p.gridZone()[1])

    @staticmethod
    def distance2D(utm1, utm2):
        """ Compute 2D Euclidean distance between two utm points.

        :param utm1: The first point.
        :type utm1: geodesy.utm.UTMPoint
        :param utm2: The second point.
        :type utm2: geodey.utm.UTMPoint

        :return: Distance in meters within the UTM XY plane. Altitudes are ignored.
        :rtype: float64
        """
        dx = utm2.easting - utm1.easting
        dy = utm2.northing - utm1.northing
        return numpy.sqrt(dx*dx + dy*dy)

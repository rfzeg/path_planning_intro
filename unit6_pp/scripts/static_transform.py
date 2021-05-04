#!/usr/bin/env python

"""
Publish a tf_static msg into a latched topic.
The tf_statiic topic is not latched, and published only one time at the beginning.
If other nodes needing that transform are not started before, they won't be able to use frames in it.
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header

if __name__ == '__main__':
  rospy.init_node('static_tf2_broadcaster', anonymous=True)

  tr_world = TransformStamped(
    header=Header(frame_id='world', stamp=rospy.Time(0)),
    child_frame_id='map',
    transform=Transform(
      translation=Vector3(0, 0, 0),
      rotation=Quaternion(0, 0, 0, 1)
    )
  )

  # Convert Lat Long to UTM, https://www.latlong.net/lat-long-utm.html
  # mcity
  # UTM Easting  277588.40
  # UTM Northing  4686714.90
  tr_map = TransformStamped(
    header=Header(frame_id='map', stamp=rospy.Time(0)),
    child_frame_id='local_map',
    transform=Transform(
      translation=Vector3(387231.33, 5818836.08, 0),
      rotation=Quaternion(0, 0, 0, 1)
    )
  )

  static_tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)
  static_tf_pub.publish(TFMessage(transforms=[tr_world]))
  static_tf_pub.publish(TFMessage(transforms=[tr_map]))
  rospy.loginfo_once("Publishing latched /tf_static")

  rospy.spin()

"""
mcity
translation=Vector3(277588, 4686714, 0)

belin zoo
translation=Vector3(387231.33 5818836.08, 0)

Since I did not find any answer on answers.ros.org or here,
and since I had a similar issue, I want to add here a very basic consideration.

If you try to open the launch file inside the osm_cartography folder, you'll find a static transform:
 
<node pkg="tf" type="static_transform_publisher" name="local_map_tf" args="622150 3362350 0 0 0 0 1 map local_map 100" />

this is the transform to the UTM reference frame for the downloaded openstreetmap.

If you try do download your own map, you need to create a new transform with the coordinates of your area.
For the lat/lon to UTM conversion, you can use this link https://www.latlong.net/lat-long-utm.html


"""
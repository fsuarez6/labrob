#!/usr/bin/env python
""" Fake localization node.  

ROS navigation code typically expects 'map' to be the base coordinate
frame.  A functioning navigation system would figure out where the
robot is in the map and then communicate that location by publishing a
transform from 'map' to 'odom'.

This node fakes that process so that the tf tree is complete..  

More info on ROS frame standards can be found here:
REP 105: http://www.ros.org/reps/rep-0105.html#coordinate-frames

Author: Francisco Suarez Ruiz
Version: 04/07/2014
Description: This script provides fake localization. 
"""
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry


class Localizer():
  """  
  Currently it just broadcasts an identity transform from 'map' to
  'odom' and uses the gazebo pose to transform 'base_footprint' to 'odom'
  """
  def __init__(self):
    rospy.init_node('localizer')
    br = tf.TransformBroadcaster() 
    self.odom_msg = Odometry()
    rospy.Subscriber('odom', Odometry, self.odom_cb)
    # Broadcast the transform at 60 HZ
    rate = rospy.Rate(60.0)
    rospy.loginfo('Localizer node successfully started')
    while not rospy.is_shutdown():
      br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'odom', 'map')
      pos = self.odom_msg.pose.pose.position
      rot = self.odom_msg.pose.pose.orientation
      br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), rospy.Time.now(), 'base_footprint', 'odom')
      rate.sleep()
  
  def odom_cb(self, msg):
    self.odom_msg = msg


if __name__ == '__main__':
  try:
    loc = Localizer()
  except rospy.ROSInterruptException:
    pass

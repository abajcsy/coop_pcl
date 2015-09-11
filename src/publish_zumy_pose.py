#!/usr/bin/python

import rospy
import tf
import numpy
from tf.transformations import *

if __name__ == '__main__':
  rospy.init_node('publish_zumy_pose')
  tf_b = tf.TransformBroadcaster()
  tf_l = tf.TransformListener()
  rate = rospy.Rate(10)
  last_pose = None
  freeze = False

  while not rospy.is_shutdown():
    if not freeze or last_pose is None:
      try:
        time_diff = rospy.Time.now() - tf_l.getLatestCommonTime('usb_cam','ar_marker_16_bundle') 
        if time_diff < rospy.Duration(0.1):
          inv_T,inv_R = tf_l.lookupTransform(
            'usb_cam','ar_marker_16_bundle',rospy.Time(0)
          )
          R = quaternion_inverse(inv_R)
          T = (quaternion_matrix(R)[0:3,0:3]).dot(-numpy.array(inv_T))
          last_pose = (T,R)
        else:
          last_pose = None
      except (
        tf.Exception, tf.LookupException,
        tf.ConnectivityException, tf.ExtrapolationException) as e:
        last_pose = None
    
    if last_pose is not None:
      tf_b.sendTransform(last_pose[0],last_pose[1],rospy.Time.now(),'usb_cam','map')
    
    rate.sleep()

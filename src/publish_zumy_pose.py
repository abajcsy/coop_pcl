#!/usr/bin/python

import rospy
import tf
import numpy
from tf.transformations import *
from coop_pcl.srv import *
from std_msgs.msg import String

class PublishZumyPose():
  def __init__(self):
    self.last_pose = None
    self.lock = False
    
  def zumy_step(self, req):
    self.lock = req.step != 0
    return StepResponse()

  def run(self):
    rospy.init_node('publish_zumy_pose')
    tf_b = tf.TransformBroadcaster()
    tf_l = tf.TransformListener()
    rate = rospy.Rate(10)
    
    rospy.Service('zumy_step', Step, self.zumy_step)

    state_pub = rospy.Publisher('zumy_state', String, queue_size=1)

    while not rospy.is_shutdown():
      if not self.lock or self.last_pose is None:
        try:
          time_diff = rospy.Time.now() - tf_l.getLatestCommonTime('usb_cam','ar_marker_16_bundle') 
          if time_diff < rospy.Duration(0.1):
            inv_T,inv_R = tf_l.lookupTransform(
              'usb_cam','ar_marker_16_bundle',rospy.Time(0)
            )
            R = quaternion_inverse(inv_R)
            T = (quaternion_matrix(R)[0:3,0:3]).dot(-numpy.array(inv_T))
            self.last_pose = (T,R)
          else:
            self.last_pose = None
        except (
          tf.Exception, tf.LookupException,
          tf.ConnectivityException, tf.ExtrapolationException) as e:
          self.last_pose = None
      
      if self.last_pose is not None:
        tf_b.sendTransform(self.last_pose[0],self.last_pose[1],rospy.Time.now(),'usb_cam','map')
      
      if self.lock:
        state_pub.publish('locked')
      else:
        state_pub.publish('unlocked')

      rate.sleep()


if __name__ == '__main__':
  pzp = PublishZumyPose()
  pzp.run()

#!/usr/bin/python

import rospy
import tf
from geometry_msgs.msg import Pose,Vector3,Quaternion,PoseArray
from std_msgs.msg import String
from coop_pcl.srv import *

class StuckCounterClass():
  def __init__(self):
    rospy.init_node('stuck_counter')

    self.last_pose = None
    self.stuck_poses = PoseArray()
    self.stuck_poses.header.frame_id = 'map'
    self.stuck = 0

  def stuck_counter(self,req):
    self.stuck ^= 1
    if self.stuck and self.last_pose is not None:
      self.stuck_poses.poses.append(self.last_pose)
    return StepResponse()

  def run(self):
    rospy.Service('stuck_step', Step, self.stuck_counter)

    tf_l = tf.TransformListener()

    rate = rospy.Rate(10)
    
    stuck_pub = rospy.Publisher('stuck_state', String, queue_size=1)
    poses_pub = rospy.Publisher('stuck_poses', PoseArray, queue_size=1)

    while not rospy.is_shutdown():  
      try:
        T,R = tf_l.lookupTransform('map','ar_marker_0_bundle',rospy.Time(0))
        self.last_pose = Pose(Vector3(*T),Quaternion(*R))
      except (
        tf.Exception, tf.LookupException,
        tf.ConnectivityException, tf.ExtrapolationException) as e:
        self.last_pose = None

      if self.stuck:
        stuck_pub.publish('stuck')
      else:
        stuck_pub.publish('moving')

      poses_pub.publish(self.stuck_poses)

      rate.sleep()

if __name__ == '__main__':
  sc = StuckCounterClass()
  sc.run()


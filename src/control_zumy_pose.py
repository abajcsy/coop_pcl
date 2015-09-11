#!/usr/bin/python

import rospy

import tf
from tf.transformations import *
from geometry_msgs.msg import PoseArray, Twist, Pose2D
from threading import Condition
from ar_track_alvar_msgs.msg import AlvarMarkers

from coop_pcl.srv import *

POS_EPS = 0.02
ANG_EPS = 0.5

MIN_VEL = 0.1
MAX_VEL = 0.15

MAX_ANG = 0.1

VEL_P = 0.2
ANG_P = 0.2

def pose_to_2d(pose):
  if type(pose) is tuple:
    T,R = pose
  else:
    p = pose.position
    T = [p.x, p.y, p.z]
    q = pose.orientation
    R = [q.x,q.y,q.z,q.w]
  R = quaternion_matrix(R)
  return numpy.array([T[0],T[1],numpy.arctan2(R[1,0],R[0,0])])

class ControlZumyPose():
  def __init__(self):
    self.last_pose = None
    self.queue = PoseArray()
    self.lock = Condition()
    self.n_markers = 0

  def add_poses(self,msg):
    self.lock.acquire()
    if len(msg.pa.poses):
      self.queue.poses += msg.pa.poses
    else:
      self.queue.poses = []
    self.lock.release()
    return AddPosesResponse()
  
  def ar_callback(self, msg):
    self.n_markers = len(msg.markers)

  def run(self):
    rospy.init_node('control_zumy_pose')
    rate = rospy.Rate(10)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    queue_pub = rospy.Publisher('queue', PoseArray, queue_size=1)
    goal_pub = rospy.Publisher('goal', Pose2D, queue_size=1)
    pose_pub = rospy.Publisher('pose', Pose2D, queue_size=1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)

    rospy.Service('add_poses', AddPoses, self.add_poses)
    tf_l = tf.TransformListener()

    while not rospy.is_shutdown():
      try:
        time_diff = rospy.Time.now() - tf_l.getLatestCommonTime('map','zumy')
        if self.n_markers > 0 and time_diff < rospy.Duration(0.1):
          pose = tf_l.lookupTransform('map','zumy',rospy.Time())
          self.last_pose = pose_to_2d(pose)
        else:
          self.last_pose = None
      except (
        tf.Exception, tf.LookupException, tf.ConnectivityException, 
        tf.ExtrapolationException) as e:
        self.last_pose = None

      cmd_vel = Twist()
     
      if self.last_pose is not None:
        pose_pub.publish(*self.last_pose)

      if len(self.queue.poses) and self.last_pose is not None:
        goal_pose = pose_to_2d(self.queue.poses[0])
        goal_pub.publish(*goal_pose)
        pos_diff = (goal_pose - self.last_pose)[0:2]
        pos_err = sum(pos_diff**2)**0.5
        ang_err = self.last_pose[2] - numpy.arctan2(pos_diff[1],pos_diff[0])
        
        print pos_err, ang_err

        vel_cmd = 0
        ang_cmd = 0
        
        if pos_err < POS_EPS:
          self.lock.acquire()
          self.queue.poses.pop(0)
          self.lock.release()
        else:
          ang_cmd = - ANG_P * ang_err
          if ang_err < ANG_EPS:
            print 'forward'
            vel_cmd = VEL_P * pos_err
          
        if vel_cmd > MAX_VEL:
          print 'clipping high vel'
          vel_cmd = MAX_VEL

        if vel_cmd != 0 and vel_cmd < MIN_VEL:
          print 'clipping low vel'
          vel_cmd = MIN_VEL

        if abs(ang_cmd) > MAX_ANG:
          print 'clipping angle'
          ang_cmd = MAX_ANG * ang_cmd/abs(ang_cmd)

        cmd_vel.linear.x = vel_cmd
        cmd_vel.angular.z = ang_cmd
      
      cmd_pub.publish(cmd_vel)
      queue_pub.publish(self.queue)

      rate.sleep()

if __name__ == '__main__':
  czp = ControlZumyPose()
  czp.run()

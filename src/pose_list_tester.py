#!/usr/bin/python

import rospy

from coop_pcl.srv import *
from geometry_msgs.msg import PoseArray, Pose

def ros_init():
  rospy.init_node('pose_list_tester')
  
def add_poses(poses):
  rospy.wait_for_service('/odroid1/add_poses')
  add_poses_proxy = rospy.ServiceProxy('/odroid1/add_poses',AddPoses)
  
  pa = PoseArray()
  for pose in poses:
    p = Pose()
    p.position.x = pose[0]
    p.position.y = pose[1]
    pa.poses.append(p)

  add_poses_proxy(pa)

if __name__ == '__main__':
  rospy.init_node('pose_list_tester')

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    poses = [p for p in raw_input().split(';') if len(p)]
    print len(poses)
    print [p.split(',') for p in poses]
    rate.sleep()


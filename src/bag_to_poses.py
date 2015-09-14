#!/usr/bin/python

import rosbag
import sys
from tf.transformations import *
import numpy
import json
import os

def check_frame(tf_msg, frame, child):
  return tf_msg.header.frame_id == frame and tf_msg.child_frame_id == child

def pose_msg_to_xyt(pose_msg):
  T = pose_msg.position
  Q = pose_msg.orientation
  R = quaternion_matrix([Q.x, Q.y, Q.z, Q.w]).T
  theta = numpy.arctan2(R[1,0],R[0,0])
  return (T.x, T.y, theta)

def tf_msg_to_H(tf_msg):
  T = tf_msg.transform.translation
  Q = tf_msg.transform.rotation
  R = quaternion_matrix([Q.x, Q.y, Q.z, Q.w]).T
  return translation_matrix([T.x, T.y, T.z]).dot(R)

def H_to_xyt(H):
  return (H[0,3],H[1,3],numpy.arctan2(H[1,0],H[0,0])) 

def full_path(path):
  if path[0] != '/':
    path = os.getcwd() + '/' + path
  return path

def get_stuck_roach_poses(cam_H, bag_file):
  bag_file = full_path(bag_file)
  
  last_stuck = None
  roach_paths = [[]]
  stuck_poses = []
  stuck_state = 'stuck'

  with rosbag.Bag(bag_file) as bag:
    for t,m,s in bag.read_messages():
      if t == '/stuck_poses':
        last_stuck = m

      elif t == '/stuck_state':
        new_state = m.data
        if stuck_state == 'stuck' and new_state == 'moving':
          roach_paths.append([])
        stuck_state = new_state
      
      elif t == '/tf' and stuck_state == 'moving':
        for tf_msg in m.transforms:
          if check_frame(tf_msg, 'usb_cam', 'ar_marker_0_bundle'):
            roach_H = tf_msg_to_H(tf_msg)
            if (abs(roach_H) < 1.1).all():
              roach_paths[-1].append(H_to_xyt(cam_H.dot(roach_H)))
    
    for path in roach_paths[:]:
      if len(path) is 0:
        roach_paths.remove(path)

    for pose_msg in last_stuck.poses:
      stuck_poses.append(pose_msg_to_xyt(pose_msg))
    
    bag.close()

  return stuck_poses, roach_paths

def get_zumy_pose(avg_file):
  avg_file = full_path(avg_file)
  
  with rosbag.Bag(avg_file) as avg:
    tfs = [m for t,m,s in avg.read_messages()][0].transforms
    map_usb_tf = None
    usb_zumy_tf = None
    for tf_msg in tfs:
      if check_frame(tf_msg, 'map', 'usb_cam'):
        map_usb_tf = tf_msg
      if check_frame(tf_msg, 'usb_cam', 'zumy'):
        usb_zumy_tf = tf_msg
    cam_H = tf_msg_to_H(map_usb_tf)
    zumy_H = cam_H.dot(tf_msg_to_H(usb_zumy_tf)) 
    zumy_pose = H_to_xyt(zumy_H) 
    avg.close()

  return (zumy_pose, cam_H)

if __name__ == '__main__':
  bag_file = sys.argv[1]
  avg_file = sys.argv[2]
  out_file = full_path(sys.argv[3])
  
  zumy_pose, cam_H = get_zumy_pose(avg_file)
  stuck_poses, roach_paths = get_stuck_roach_poses(cam_H, bag_file)

  data = {
    'cam_h': cam_H.tolist(),
    'stuck': stuck_poses, 
    'roach': roach_paths, 
    'zumy': zumy_pose
  }

  f = open(out_file,'w')
  json.dump(data,f)
  f.close()

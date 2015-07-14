#!/usr/bin/env python

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from apriltags_ros.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
import time
import rospy
import sys

def Rater():
  def callback(message):
    global TimA
    global Begin
    global sumDelT
    global sumSam
    for det in message.transforms:
      #roboX=det.pose.pose.position.x
      #roboX=det.transform.translation.x
      #TimB=det.header.stamp
      TimB=rospy.Time.now()
      delT=TimB-TimA
      sumDelT += delT
      sumSam += 1
      avDelT= (TimB-Begin)/sumSam
      print("ID: %s"%det.child_frame_id + ", delT: %d" % delT.secs + ".%d " % delT.nsecs)
      print("sum = %d" % sumDelT.secs + ".%d " % sumDelT.nsecs)
      print("#samples = %d" % sumSam + ", average delT = %d" % avDelT.secs + ".%d " % delT.nsecs)
      TimA=TimB
  #rospy.Subscriber('/tag_detections',AprilTagDetectionArray, callback)
  rospy.Subscriber('/tf',TFMessage, callback)
  while not rospy.is_shutdown():
    # Delay
    time.sleep(0.6)


if __name__ == '__main__':
  try:
    rospy.init_node('Rater', anonymous=True)
    TimA=rospy.Time.now()
    Begin=rospy.Time.now()
    sumDelT=rospy.Duration(0)
    sumSam=0
    Rater()
  except rospy.ROSInterruptException: pass

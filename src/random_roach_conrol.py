#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from coop_pcl.srv import *
import random

MAX_VEL = 0.4
FWD_VEL = MAX_VEL
REV_VEL = -MAX_VEL

MAX_ANG = 0.5
FWD_ANG = MAX_ANG
REV_ANG = -MAX_ANG

MIN_PERIOD = 10
MAX_PERIOD = 50

phases = ((FWD_VEL,0),(0,FWD_ANG),(REV_VEL,0),(0,REV_ANG))

class RandomRoachController():
  def __init__(self):
    self.state = 'idle'
    self.count = 0
    self.phase = 0

  def roach_step(self, req):
    if self.state == 'idle':
      if req.step == 1:
        self.state = 'done'
      else:
        self.state = 'moving'
    
    elif self.state == 'moving':
      if req.step == 1:
        self.state = 'done'
      else:
        self.state = 'idle'
    
    return StepResponse()

  def run(self):
    rospy.init_node('random_roach_controller')
    rate = rospy.Rate(10)
    state_pub = rospy.Publisher('roach_state', String, queue_size=1)
    cmd_pub = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)

    rospy.Service('roach_step', Step, self.roach_step)

    while not rospy.is_shutdown():  
      cmd = Twist()
     
      if self.state == 'moving':
        if self.count == 0:
          self.phase = (self.phase + 1) % len(phases)
          self.count = random.randrange(MIN_PERIOD,MAX_PERIOD)
        else:
          self.count -= 1

        cmd.linear.x = phases[self.phase][0]
        cmd.angular.z = phases[self.phase][1]
      
      cmd_pub.publish(cmd)

      state_pub.publish('%s:%d:%d' % (self.state,self.phase,self.count))

      rate.sleep()

if __name__ == '__main__':
  rrc = RandomRoachController()
  rrc.run()

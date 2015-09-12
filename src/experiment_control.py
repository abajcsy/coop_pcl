#!/usr/bin/python

import rospy
from std_msgs.msg import String
from coop_pcl.srv import *

def null_proxy(step):
  return

class ExperimentControl():
  def __init__(self):
    self.state = 'init'
    self.proxies = {
      'zumy':null_proxy,
      'stuck':null_proxy,
      'cloud':null_proxy,
      'roach':null_proxy
    }

  def step_state(self, req):
    # After init, lock zumy, start cloud publisher, and roach controller
    if self.state == 'init':
      self.proxies['zumy'](1)
      self.proxies['cloud'](0)
      self.proxies['roach'](0)
      self.state = 'moving'

    # If step == 1, end experiment and save, otherwise toggle to stuck
    elif self.state == 'moving':
      if req.step == 1:
        self.state = 'saving'
      else:
        self.proxies['stuck'](0)
        self.proxies['roach'](0)
        self.state = 'stuck'

    # If step == 1, end experiment and save, otherwise toggle to moving
    elif self.state == 'stuck':
      if req.step == 1:
        self.state = 'saving'
      else:
        self.proxies['stuck'](0)
        self.proxies['roach'](0)
        self.state = 'moving'

    return StepResponse()

  def try_setting_proxy(self, proxy_name, service_name):
    if self.proxies[proxy_name] is null_proxy:
      try:
        rospy.wait_for_service(service_name,0.01)
        self.proxies[proxy_name] = rospy.ServiceProxy(service_name, Step)
        print 'Experiment Control set proxy for %s' % service_name
      except rospy.exceptions.ROSException:
        pass
        
  def run(self):
    rospy.init_node('experiment_control')

    state_pub = rospy.Publisher('experiment_state', String, queue_size=1)
    
    rate = rospy.Rate(10)

    rospy.Service('experiment_step', Step, self.step_state)
    
    while not rospy.is_shutdown():
      if self.state == 'init':
        self.try_setting_proxy('zumy', 'zumy_step')
        self.try_setting_proxy('cloud', 'cloud_step')
        self.try_setting_proxy('roach', 'roach_step')
        self.try_setting_proxy('stuck', 'stuck_step')
    
      # Tell cloud to save PCL file, and stop roach controller
      elif self.state == 'saving':
        print 'Experiment Control saving'

        self.proxies['cloud'](1)
        self.proxies['roach'](1)
        self.state = 'done'
      
      state_pub.publish(self.state)
      rate.sleep()

if __name__ == '__main__':
  ec = ExperimentControl()
  ec.run()

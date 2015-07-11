#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ar_tf_listener");

  ros::NodeHandle node;

  //ros::Publisher turtle_vel = node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  tf::TransformListener transform_listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      // can't always guaruntee that it will be called ar_marker_2?
      transform_listener.lookupTransform("/usb_cam", "/ar_marker_2",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    double centerX = transform.getOrigin().x();
    double centerY = transform.getOrigin().y(); 

    

    rate.sleep();
  }
  return 0;
};

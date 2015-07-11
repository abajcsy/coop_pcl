#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void cloud_cb()
{
  PointCloud::Ptr msg(new PointCloud);
  msg->header.frame_id = "/usb_cam";
  msg->height = 1;
  msg->width = 12; // note: always make width the size of the total number of pts in ptcloud

  tf::TransformListener transform_listener;
  
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/usb_cam", "/ar_marker_2", now, ros::Duration(3.0));

      // can't always guaruntee that it will be called ar_marker_2?
      transform_listener.lookupTransform("/usb_cam", "/ar_marker_2", ros::Time(0), transform_listener);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    double centerX = transform_listener.getOrigin().x();
    double centerY = transform_listener.getOrigin().y(); 
    double centerZ = transform_listener.getOrigin().z(); 
    
    msg->points.push_back (pcl::PointXYZ(centerX, centerY, centerZ));

    msg->header.stamp = ros::Time::now().toNSec();
    pub.publish(msg);
  }

}

int main(int argc, char** argv)
{
   // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
 
  // Create ROS publisher for the output point cloud
  ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);

  ros::spin();
}

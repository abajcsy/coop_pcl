#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <string>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
   // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
 
  // Create ROS publisher for the output point cloud
  ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);

  PointCloud::Ptr msg(new PointCloud);
  msg->header.frame_id = "usb_cam";
  msg->height = 1;
  msg->width = 30000; // note: always make width the size of the total number of pts in ptcloud
  //msg->is_dense = false;
  msg->points.resize(msg->width * msg->height);

  tf::TransformListener transform_listener;
  
  string ar_marker = "ar_marker_0";

  int i = 0;
  ros::Rate loop_rate(4);
  while (nh.ok() && i < msg->points.size())
  {
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(3.0));

	  cout << "Looking up tf from usb_cam to ar_marker...\n";
      // can't always guaruntee that it will be called ar_marker?
      transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    cout << "Found tf from usb_cam to ar_marker...\n";
	double x = 0.10;
	while(x >= -0.10){
		double y = -0.10;
		while(y <= 0.10){
			// get stamped pose wrt ar_marker
			tf::Stamped<tf::Pose> corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, 0.0)),ros::Time(0), ar_marker);
			tf::Stamped<tf::Pose> transformed_corner;
			// transform pose wrt usb_cam
			transform_listener.transformPose("usb_cam", corner, transformed_corner);
			// push xyz coord of pt to point cloud
			double tf_x = transformed_corner.getOrigin().x();
			double tf_y = transformed_corner.getOrigin().y();
			double tf_z = transformed_corner.getOrigin().z();
			//msg->points.push_back (pcl::PointXYZ(tf_x, tf_y, tf_z));
			msg->points[i].x = tf_x;
			msg->points[i].y = tf_y;
			msg->points[i].z = tf_z;
			
			i+=1;
			cout << "Added (" << tf_x << "," << tf_y << "," << tf_z << ")\n";
			y += 0.05;
		}
		x -= 0.05;
	}

    msg->header.stamp = ros::Time::now().toNSec();
    pub.publish(msg);
  }

  ros::spin();
}

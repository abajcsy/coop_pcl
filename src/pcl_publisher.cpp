#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void publishPCL(ros::NodeHandle nh, ros::Publisher pub) {
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = "usb_cam";
  cloud->height = 1;
  cloud->width = 15000; // note: always make width the size of the total number of pts in ptcloud
  //cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  tf::TransformListener transform_listener;
  
  string ar_marker = "ar_marker_1"; // argv[1]

  int idx_pts = 0;
  ros::Rate loop_rate(4);
  while (nh.ok() && idx_pts < cloud->points.size()) {
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(3.0));

	  cout << "Looking up tf from usb_cam to " << ar_marker << "...\n";
      // can't always guarantee that it will be called any particular ar_marker_#?
      transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    cout << "Found tf from usb_cam to " << ar_marker << "...\n";
	double x = 0.05;
	while(x >= -0.05){
		double y = -0.02;
		while(y <= 0.02){
			// get stamped pose wrt ar_marker
			tf::Stamped<tf::Pose> corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, 0.0)),ros::Time(0), ar_marker);
			tf::Stamped<tf::Pose> transformed_corner;
			// transform pose wrt usb_cam
			transform_listener.transformPose("usb_cam", corner, transformed_corner);
			// push xyz coord of pt to point cloud
			double tf_x = transformed_corner.getOrigin().x();
			double tf_y = transformed_corner.getOrigin().y();
			double tf_z = transformed_corner.getOrigin().z();

			cloud->points[idx_pts].x = tf_x;
			cloud->points[idx_pts].y = tf_y;
			cloud->points[idx_pts].z = tf_z;
			
			idx_pts+=1;
			cout << "Added (" << tf_x << "," << tf_y << "," << tf_z << ")\n";
			y += 0.02;
		}
		x -= 0.02;
	}
    cloud->header.stamp = ros::Time::now().toNSec();
	pub.publish(cloud);
  }

  // if program was killed or point cloud filled filled 
  if(!nh.ok() || idx_pts >= cloud->points.size()){	// TODO: This is still a hack-y way to save the point cloud. Integrate asynchronous keyboard input command to stop data collection and write pc
  	cout << "Finished gathering and publishing point cloud.\n";
  	string filename = "test_pcd.pcd";
  	pcl::io::savePCDFileASCII(filename, *cloud);
  	cout << "Saved " << idx_pts << " points out of total point cloud space of " << cloud->points.size() << " to " << filename << "\n";
  }
}

int main(int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;

  // Create ROS publisher for the output point cloud
  ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
  
  // Call point cloud publisher function
  publishPCL(nh, pub);
 
  ros::spin();
}

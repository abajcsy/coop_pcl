#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>

#include <stdio.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreePC;

/* This function continously updates the PointCloud structure as well as the octree based on the 
 * AR bundles that are being tracked by ar_tracker_alvar. The size of each individual point cloud 
 * being projected into the pose of the VelociRoACH is 0.04 x 0.1 m (approx. the size of the RoACH).
 * The Octree is continuously being updated with new data points and is published for visualization 
 * and queries through rviz and other ROS nodes.
 */
void publishPointCloud(ros::NodeHandle nh, ros::Publisher pub_cloud, ros::Publisher pub_octree_marker, PointCloud::Ptr cloud, OctreePC octree) {
  tf::TransformListener transform_listener;
  
  string ar_marker = "ar_marker_1"; 

  int idx_pts = 0;
  ros::Rate loop_rate(4);
  while (nh.ok() && idx_pts < cloud->width*cloud->height) {
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(3.0));
	  cout << "Looking up tf from usb_cam to " << ar_marker << "...\n";
      transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    cout << "Found tf from usb_cam to " << ar_marker << "...\n";
	double x = 0.05;
	while(x >= -0.05 ){
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
		
			// update octree cloud
			octree.setInputCloud(cloud);

			cout << "Added (" << tf_x << "," << tf_y << "," << tf_z << ")\n";
			idx_pts+=1;
			y += 0.02;
		}
		x -= 0.02;
	}
	cout << "point cloud size: " << cloud->points.size() << ", idx_pts: " << idx_pts <<endl;
	cout << "cloud->width*cloud->height = " << cloud->width*cloud->height << endl;
    cloud->header.stamp = ros::Time::now().toNSec();
	pub_cloud.publish(cloud);
  }

  // TODO: This is still a hack-y way to save the point cloud. Integrate asynchronous keyboard input command to stop data collection and write pc.
  // if program was killed or point cloud filled filled 
  if(idx_pts >= cloud->points.size()){	
  	cout << "Finished gathering and publishing point cloud.\n";
  	string filename = "/home/humanoid/ros_workspace/src/coop_pcl/resources/test_pcd2.pcd";
  	pcl::io::savePCDFileASCII(filename, *cloud);
  	cout << "Saved " << idx_pts << " points out of total point cloud space of " << cloud->points.size() << " to " << filename << "\n";
	exit(1);
  }
}

int main(int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;

  // Create ROS publisher for the output point cloud
  ros::Publisher pub_cloud = nh.advertise<PointCloud>("points2", 1);

  // Create ROS publisher for the output octree
  //ros::Publisher pub_octree_marker_array = nh.advertise<visualization_msgs::Marker>("visualization_marker_array", 100);
  ros::Publisher pub_octree_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

  // Create point cloud
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = "usb_cam";
  cloud->height = 1;
  cloud->width = 15000; // note: always make width the size of the total number of pts in ptcloud
  //cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  // resolution = size (side length) of a single voxel at the lowest level in the octree (lowest level->smallest voxel)
  // considering the scale of the velociroach map (where the velociroach is 0.04 x 0.1 m) we want very fine resolution
  float resolution = 0.001f; 

  // Create empty octree where octree = vector of point indicies with leaf nodes
  OctreePC octree(resolution); 
  octree.setInputCloud(cloud); // set octree's input cloud

  // Call point cloud publisher function
  publishPointCloud(nh, pub_cloud, pub_octree_marker, cloud, octree);
 
  ros::spin();
}

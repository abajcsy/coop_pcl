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
#include <termios.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreePC;

static struct termios oldt, newt;

/* Implements a non-blocking getchar() in Linux. Function modifying the terminal settings to 
 * disable input buffering. 
 */
int getch() {
  tcgetattr(STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~ICANON;                 // disable buffering  
  newt.c_lflag &= ~ECHO; 
  newt.c_lflag &= ~ISIG;   
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int input = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return input;
}

/* Restores the original keyboard settings when the program exits, or else the terminal 
 * will behave oddly. 
 */
void resetKeyboardSettings() {
	tcsetattr(0, TCSANOW, &oldt);
}

/* This function continously updates the PointCloud structure as well as the octree based on the 
 * AR bundles that are being tracked by ar_tracker_alvar. The size of each individual point cloud 
 * being projected based on the pose of the VelociRoACH is 0.04 x 0.1 m (approx. the size of the RoACH).
 * The Octree is continuously being updated with new data points and is published for visualization 
 * and queries through rviz and other ROS nodes.
 */
void publishPointCloud(ros::NodeHandle nh, ros::Publisher pub_cloud, PointCloud::Ptr cloud, OctreePC octree, std::string pcd_filename) {
  tf::TransformListener transform_listener;
  
  string ar_marker = "ar_marker_1"; 

  int idx_pts = 0;
  ros::Rate loop_rate(4);
  int input = 'c';
  cout << "Press 'q' to quit data collection and save point cloud.\n";
  while (nh.ok() && idx_pts < cloud->width*cloud->height) {
	input = getch();   // check if user pressed q to quit data collection
	if (input == 'q'){
		break;
	}

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
	cout << "Point cloud size: " << cloud->points.size() << ", idx_pts: " << idx_pts <<endl;
	cout << "Cloud->width * cloud->height = " << cloud->width*cloud->height << endl;
    cloud->header.stamp = ros::Time::now().toNSec();
	pub_cloud.publish(cloud);
  }

  // if program was terminated or point cloud filled, save out the point cloud and end program 
  if(input == 'q' || idx_pts >= cloud->points.size()){	
  	cout << "Finished gathering and publishing point cloud.\n";
  	pcl::io::savePCDFileASCII(pcd_filename, *cloud);
  	cout << "Saved " << idx_pts << " points out of total point cloud space of " << cloud->points.size() << " to " << pcd_filename << "\n";
	resetKeyboardSettings();
	cout << "Reset keyboard settings and shutting down.\n";
	ros::shutdown();
  }
}

/* Initializes ROS node and runs main functionality
 * NOTE: requires three arguments:
 *		argv[1] = name of pcd file to save point cloud to
 *		argv[2] = resolution of point cloud
 *		argv[3] = width of point cloud (aka. total number of points allowed in cloud)
 */
int main(int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;

  // Sanity check
  if(argc != 4){
	ROS_ERROR("Not enough arguments! Usage example: pcl_publisher test_pcd.pcd 0.01 15000");
	ros::shutdown();
  }

  string pcd_filename = argv[1];
  cout << "PCD Output Filename: " << pcd_filename << endl;

  // Create ROS publisher for the output point cloud
  ros::Publisher pub_cloud = nh.advertise<PointCloud>("points2", 1);

  // Create point cloud
  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = "usb_cam";
  cloud->height = 1;
  cloud->width = atoi(argv[3]); 
  cloud->points.resize(cloud->width * cloud->height);
  cout << "Cloud Size: " << cloud->width << endl;

  // Resolution = size (side length) of a single voxel at the lowest level in the octree (lowest level->smallest voxel)
  // Considering the scale of the velociroach map (where the velociroach is 0.04 x 0.1 m) we want very fine resolution
  float resolution = strtof(argv[2], NULL); 
  cout << "Cloud Resolution: " << resolution << endl;

  // Create empty octree where octree = vector of point indicies with leaf nodes
  OctreePC octree(resolution); 
  octree.setInputCloud(cloud); // set octree's input cloud

  // Call point cloud publisher function
  publishPointCloud(nh, pub_cloud, cloud, octree, pcd_filename);
 
  ros::spin();
}

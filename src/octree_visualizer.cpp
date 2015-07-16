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
#include <math.h> 

#include <stdio.h>
#include <stdlib.h> 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreePC;

using namespace std;

// TODO: should be able to pass in filename of point cloud, resolution of octree
int main(int argc, char** argv){
	
	// Initialize ROS
  	ros::init (argc, argv, "octree_vis");
  	ros::NodeHandle nh;

	cout << "got here." << endl;
	cout << argc << endl;
	for(int i = 0; i < argc; i++){
		cout << argv[i] << endl;
	}

	if(argc != 2){
		PCL_ERROR ("No octree resolution! Usage example: octree_visualizer 0.01 \n");
		exit(1);
	}

	// load cloud
	PointCloud::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	string filename = "/home/humanoid/ros_workspace/src/coop_pcl/resources/test_pcd2.pcd";
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load the file
  	{
    	PCL_ERROR ("Couldn't read pcd file! \n");
    	return (-1);
  	}
  	cout << "Loaded cloud with width*height = " << cloud->width * cloud->height << endl;

	// convert cloud to octree
	float resolution = strtof(argv[1], NULL); //0.1f; 0.001f;
	cout << resolution << endl;
    OctreePC octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	cout << "Done setting up octree from cloud." <<endl;

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxelCenters;
	voxelCenters.clear();
	octree.getOccupiedVoxelCenters(voxelCenters);
	double voxelSideLen = sqrt(octree.getVoxelSquaredSideLen()); 

	ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	cout << "Done setting up marker publisher." <<endl;

	// set up MarkerArray for visualizing in RVIZ
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(voxelCenters.size());
	for(size_t i = 0; i < voxelCenters.size(); i++){
        double s = voxelSideLen; 
        double x = voxelCenters[i].x;
        double y = voxelCenters[i].y;
        double z = voxelCenters[i].z;

		cout << "s: " << s << ", x: " << x << ", y: " << y << ", z: " << z << endl;		

		marker_array_msg.markers[i].header.frame_id = "map";		// TODO: should be map, but can change
		marker_array_msg.markers[i].header.stamp = ros::Time();
		marker_array_msg.markers[i].ns = "my_namespace";
		marker_array_msg.markers[i].id = i; 
		marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;
		marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

		marker_array_msg.markers[i].pose.position.x = x;
		marker_array_msg.markers[i].pose.position.y = y;
		marker_array_msg.markers[i].pose.position.z = z;
		marker_array_msg.markers[i].pose.orientation.x = 0.0;		// TODO: correct the orientation (?)
		marker_array_msg.markers[i].pose.orientation.y = 0.0;
		marker_array_msg.markers[i].pose.orientation.z = 0.0;
		marker_array_msg.markers[i].pose.orientation.w = 1.0;

		marker_array_msg.markers[i].scale.x = s;
		marker_array_msg.markers[i].scale.y = s;
		marker_array_msg.markers[i].scale.z = s;

		marker_array_msg.markers[i].color.a = 1.0; // Don't forget to set the alpha!
		marker_array_msg.markers[i].color.r = 0.0;
		marker_array_msg.markers[i].color.g = 1.0;
		marker_array_msg.markers[i].color.b = 0.0;
	}

	while(nh.ok()){
		pub_marker.publish(marker_array_msg);
	}

	cout << "done." << endl;

	ros::spin();
}

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/* Publishes marker for RVIZ at location (x,y,z) with (r,g,b) color
 */
visualization_msgs::Marker publishMarker(geometry_msgs::Point pt, double scale, double r, double g, double b){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pt.x;
	marker.pose.position.y = pt.y;
	marker.pose.position.z = pt.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = 0.5;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	return marker;
}	

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "planar_seg");
  ros::NodeHandle nh;
  ros::Publisher cloud_pub = nh.advertise<PointCloud>("points2", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  PointCloud::Ptr cloud(new PointCloud);

  // Fill in the cloud data
  cloud->header.frame_id = "map";
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
	cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud->points[i].z = 1.0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
	std::cerr << "    " << cloud->points[i].x << " "
		                << cloud->points[i].y << " "
		                << cloud->points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		                              << coefficients->values[1] << " "
		                              << coefficients->values[2] << " " 
		                              << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
	std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		                                       << cloud->points[inliers->indices[i]].y << " "
		                                       << cloud->points[inliers->indices[i]].z << std::endl;

  ros::Rate loop_rate(4);
  // publish cloud
  while(nh.ok()){
	  cloud->header.stamp = ros::Time::now().toNSec();
	  cloud_pub.publish(cloud);

	  geometry_msgs::Point pt;
	  pt.x = 0; pt.y = 0; pt.z = 1;
	  visualization_msgs::Marker marker = publishMarker(pt, 5.0, 0.0, 1.0, 0.0);
	  marker_pub.publish(marker);
	  ros::spinOnce(); 
   }

}

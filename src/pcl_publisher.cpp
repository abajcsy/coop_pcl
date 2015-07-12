#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

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
  msg->width = 100; // note: always make width the size of the total number of pts in ptcloud

  tf::TransformListener transform_listener;
  
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      transform_listener.waitForTransform("usb_cam", "ar_marker_2", now, ros::Duration(3.0));

      // can't always guaruntee that it will be called ar_marker_2?
      transform_listener.lookupTransform("usb_cam", "ar_marker_2", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // public point cloud with the size and shape of the VelociRoACH 
    /*double centerX = transform.getOrigin().x();
    double centerY = transform.getOrigin().y(); 
    double centerZ = transform.getOrigin().z(); 

    int arHeight = 7; // ar tag is 3 in or ~7.62 cm
    int arWidth = 7;
    for(int i=0; i < arHeight; i++){
		for(int j=0;j < arWidth; j++){
		
	}
    }*/

    double centerX = transform.getOrigin().x();
    double centerY = transform.getOrigin().y(); 
    double centerZ = transform.getOrigin().z();

	tf::Stamped<tf::Pose> corner_1(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.10, -0.10, 0.0)),ros::Time(0), "ar_marker_2");
	tf::Stamped<tf::Pose> corner_2(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.10, 0.10, 0.0)),ros::Time(0), "ar_marker_2");
	tf::Stamped<tf::Pose> corner_3(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.10, -0.10, 0.0)), ros::Time(0), "ar_marker_2");
	tf::Stamped<tf::Pose> corner_4(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.10, 0.10, 0.0)), ros::Time(0), "ar_marker_2");

	tf::Stamped<tf::Pose> transformed_corner_1;
	transform_listener.transformPose("usb_cam", corner_1, transformed_corner_1);
	tf::Stamped<tf::Pose> transformed_corner_2;
	transform_listener.transformPose("usb_cam", corner_2, transformed_corner_2);
	tf::Stamped<tf::Pose> transformed_corner_3;
	transform_listener.transformPose("usb_cam", corner_3, transformed_corner_3);
	tf::Stamped<tf::Pose> transformed_corner_4;
	transform_listener.transformPose("usb_cam", corner_4, transformed_corner_4);
    
	// push 4 corners of AR tag to pt cloud
    msg->points.push_back (pcl::PointXYZ(transformed_corner_1.getOrigin().x(), transformed_corner_1.getOrigin().y(), transformed_corner_1.getOrigin().z()));

	msg->points.push_back (pcl::PointXYZ(transformed_corner_2.getOrigin().x(), transformed_corner_2.getOrigin().y(), transformed_corner_2.getOrigin().z()));

	msg->points.push_back (pcl::PointXYZ(transformed_corner_3.getOrigin().x(), transformed_corner_3.getOrigin().y(), transformed_corner_3.getOrigin().z()));

	msg->points.push_back (pcl::PointXYZ(transformed_corner_4.getOrigin().x(), transformed_corner_4.getOrigin().y(), transformed_corner_4.getOrigin().z()));

    // push origin of AR tag to pt cloud
	msg->points.push_back (pcl::PointXYZ(centerX,centerY,centerZ));

    msg->header.stamp = ros::Time::now().toNSec();
    pub.publish(msg);
  }

  ros::spin();
}

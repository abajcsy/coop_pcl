#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <stdio.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudProjector {
  private: 
	ros::NodeHandle nh_;
	ros::Publisher cloud_pub_;
	ros::Publisher proj_cloud_pub_;	
		
	PointCloud::Ptr cloud;
	PointCloud::Ptr cloud_projected;

  public: 
	CloudProjector(ros::NodeHandle &nh){
		nh_ = nh;
		cloud_pub_ = nh_.advertise<PointCloud>("points2", 1);
		proj_cloud_pub_ = nh_.advertise<PointCloud>("proj_points2", 1);

		// Fill in the cloud data
		PointCloud::Ptr tmp_cloud(new PointCloud); 	
		cloud = tmp_cloud;
		cloud->width  = 15000;
		cloud->height = 1;
		cloud->header.frame_id = "usb_cam";
		cloud->points.resize (cloud->width * cloud->height);

		PointCloud::Ptr tmp_proj_cloud(new PointCloud); 	
		cloud_projected = tmp_proj_cloud;
		cloud_projected->header.frame_id = "usb_cam";
	}

	/* Projects points within input point cloud onto the plane specified by ax + by + cz + e=0
	 */
	int project_inliers(){
		/* fill cloud with test points
		for (size_t i = 0; i < cloud->points.size(); ++i) {
			cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
			cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
			cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
		}*/

		// Load point cloud from file
		string pcd_filepath = "/home/andrea/ros_workspace/src/coop_pcl/data/pcd/plane_rosbag.pcd";
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile (pcd_filepath, cloud_blob);
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
		cloud->header.frame_id = "usb_cam";

		cout << "Finished reading input point cloud..." << endl;

		/*std::cerr << "Cloud before projection: " << std::endl;
		for (size_t i = 0; i < cloud->points.size (); ++i)
		std::cerr << "    " << cloud->points[i].x << " " 
				<< cloud->points[i].y << " " 
				<< cloud->points[i].z << std::endl;
		pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);*/

		// Create a set of planar coefficients with X=Y=0,Z=1
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		coefficients->values.resize (4);
		coefficients->values[0] = 0;
		coefficients->values[1] = 1.0;
		coefficients->values[2] = 0;
		coefficients->values[3] = 0;

		// Create the filtering object
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		proj.setInputCloud (cloud);
		proj.setModelCoefficients (coefficients);
		proj.filter (*cloud_projected);
		cloud_projected->header.frame_id = "usb_cam";

		/*std::cerr << "Cloud after projection: " << std::endl;
		for (size_t i = 0; i < cloud_projected->points.size (); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " " 
				<< cloud_projected->points[i].y << " " 
				<< cloud_projected->points[i].z << std::endl;*/
		cout << "Finished projecting points..." << endl;

		pcl_conversions::toPCL(ros::Time::now(), cloud_projected->header.stamp);

		string pcd_filename_out = "/home/andrea/ros_workspace/src/coop_pcl/data/pcd/plane_projected.pcd";
		pcl::io::savePCDFileASCII(pcd_filename_out, *cloud_projected);
		cout << "Saved projected point cloud to " << pcd_filename_out << endl;				

		return (0);
	}

	void run(){
		project_inliers();

		ros::Rate loop_rate(2);
		while(nh_.ok()){
			// publish both clouds
			cloud_pub_.publish(cloud);
			proj_cloud_pub_.publish(cloud_projected);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}

};


int main(int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "cloud_projection");
  ros::NodeHandle nh;

  CloudProjector driver(nh);
  driver.run();
}


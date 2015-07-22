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

#include <coop_pcl/exploration_info.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeSearch;

static struct termios oldt, newt;

class CloudGenerator {

	private: 
		ros::NodeHandle nh_;
		ros::Publisher cloud_pub_;
		ros::Publisher cmd_vel_pub_;
		ros::Publisher explore_info_;
		PointCloud::Ptr cloud_;
		OctreeSearch *octreeSearch_;

		/* Returns a random double between fMin and fMax.
		 */
		double randDouble(double fMin, double fMax) {
			double f = (double)rand() / RAND_MAX;
			return fMin + f * (fMax - fMin);
		}

		/* Implements a non-blocking getchar() in Linux. Function modifying the terminal settings to 
		 * disable input buffering. 
		 * TODO: This is a hack. 
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


	public:
		/* Resolution = size (side length) of a single voxel at the lowest level in the octree (lowest level->smallest voxel)
	     * Considering the scale of the velociroach map (where the velociroach is 0.04 x 0.1 m) we want very fine resolution
		 */
		CloudGenerator(ros::NodeHandle &nh, int width, float resolution){
			nh_ = nh;
			cloud_pub_ = nh_.advertise<PointCloud>("points2", 1);
			//explore_info_ = nh_.advertise<coop_pcl::exploration_info>("navig", 1);
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);    // TODO: change robot1 to robotN where N is the Nth robot that we are controlling	
			
			PointCloud::Ptr cloud(new PointCloud); 	
			cloud_ = cloud;																// TODO: this isn't good practice, but can't get it to work otherwise...
			cloud_->header.frame_id = "usb_cam";
		    cloud_->height = 1;
		    cloud_->width = width; 
		    cloud_->points.resize(cloud_->width * cloud_->height);

			octreeSearch_ = new OctreeSearch(resolution);
			octreeSearch_->setInputCloud(cloud_);
		}

		/* Returns vector of indices of all points in point cloud that are within radius of searchPoint.
		 */
		vector<int> pointIdxRadiusSearch(pcl::PointXYZ searchPoint, float radius){
		  vector<int> pointIdxRadiusSearch;
		  vector<float> pointRadiusSquaredDistance;

		  cout << "Neighbors within radius search at (" << searchPoint.x << " " << searchPoint.y  << " " << searchPoint.z << ") with radius=" << radius << endl;

		  if (octreeSearch_->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			cout << "Found points within " << radius << "of search point." << endl;
		  }
		  return pointIdxRadiusSearch;
		}

		/* Runs point cloud publishing and VelociRoACH controls.
		 * This function continously updates the PointCloud structure as well as the octree based on the 
		 * AR bundles that are being tracked by ar_tracker_alvar. The size of each individual point cloud 
		 * being projected based on the pose of the VelociRoACH is 0.04 x 0.1 m (approx. the size of the RoACH).
		 * The Octree is continuously being updated with new data points and is published for visualization 
		 * and queries through rviz and other ROS nodes.
		 */
		void run(std::string pcd_filename){
			tf::TransformListener transform_listener;
			string ar_marker = "ar_marker_1"; 
			ros::Rate loop_rate(4);

			int input = 'c';
			int num_pts = 0;
			int maxCloudSize = cloud_->width*cloud_->height;

			// we will be sending commands of type "Twist"
			// geometry_msgs::Twist base_cmd;

			cout << "Press 'q' to quit data collection and save point cloud.\n";

			while (nh_.ok() && num_pts < maxCloudSize && input != 'q') {
				input = getch();   // check if user pressed q to quit data collection
				if (input == 'q'){
					break;
				}
				tf::StampedTransform transform;
				try{
				  ros::Time now = ros::Time::now();
				  transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(5.0));
				  cout << "Looking up tf from usb_cam to " << ar_marker << "...\n";
				  transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  ros::Duration(1.0).sleep();
				}

				/****************** MOVE VELOCIROACH *******************
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

				double randLinX = randDouble(0.0, 1.0); srand(time(NULL));
				double randAngle = randDouble(-1.0, 1.0); srand(time(NULL));

				cout << "randLinX: " << randLinX << ", randAngle: " << randAngle << endl;

				base_cmd.linear.x = randLinX;   // move forward
				base_cmd.angular.z = randAngle;   // move left or right
		        cmd_vel_pub_.publish(base_cmd);	
				/*********************************************************/


				/********* PUBLISH POINT CLOUD UNDER VELOCIROACH *********/
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
			
						cloud_->points[num_pts].x = tf_x;
						cloud_->points[num_pts].y = tf_y;
						cloud_->points[num_pts].z = tf_z;

						// update octree cloud pointer and add points to octree
						octreeSearch_->setInputCloud(cloud_);
					 	octreeSearch_->addPointsFromInputCloud();

						//cout << "Added (" << tf_x << "," << tf_y << "," << tf_z << ")\n";
						num_pts+=1;
						y += 0.02;
					}
					x -= 0.02;
				}
				cloud_->header.stamp = ros::Time::now().toNSec();
				cloud_pub_.publish(cloud_);
				cout << "Point cloud size: " << cloud_->points.size() << ", num_pts: " << num_pts <<endl;
				cout << "Cloud->width * cloud->height = " << maxCloudSize << endl;
				/*********************************************************/
				
			}

			// if program was terminated or point cloud filled, save out the point cloud and end program 
			if(input == 'q' || num_pts >= cloud_->points.size()) {	
				cout << "Finished gathering and publishing point cloud.\n";

				if(pcd_filename.compare("NULL") != 0){
					pcl::io::savePCDFileASCII(pcd_filename, *cloud_);
				  	cout << "Saved " << num_pts << " points out of total point cloud space of " << cloud_->points.size() << " to " << pcd_filename << endl;				
				}else{
					cout << "User specified NOT to save point cloud.\n";
				}
				resetKeyboardSettings();
				cout << "Reset keyboard settings and shutting down.\n";
				ros::shutdown();
			}
		}
};


/* Initializes ROS node and runs main functionality
 * NOTE: requires three arguments:
 *		argv[1] = name of pcd file to save point cloud to OR type NULL and the pcd file will NOT be saved
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
  float resolution = strtof(argv[2], NULL);  
  int width = atoi(argv[3]); 

  cout << "PCD Output Filename: " << pcd_filename << endl;
  cout << "Cloud Size: " << width << endl;
  cout << "Octree Resolution: " << resolution << endl;

  CloudGenerator driver(nh, width, resolution);
  driver.run(pcd_filename);
 
  ros::spin();
}

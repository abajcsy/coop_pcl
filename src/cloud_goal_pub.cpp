#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>

#include <stdio.h>
#include <termios.h>
#include <math.h> 
#include <limits.h>

#include <coop_pcl/exploration_info.h>

// VelociRoACH measurements in meters
#define ROACH_W 0.04
#define ROACH_H 0.10

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeSearch;

static struct termios oldt, newt;

/* THIS CLASS IS MEANT TO BE USED TOGETHER WITH ROACH_CONTROL_PUB.CPP
 * Together, cloud_goal_pub.cpp publishes goal points for the roach
 * to navigate to while roach_control_pub.cpp publishes the roaches state
 * in navigating to a given goal (i.e. if it suceeded at getting to the
 * goal and needs a new goal or if it is still navigating).
 * cloud_goal_pub.cpp listens to if the roach has gotten to the goal point
 * and if it does then it publishes a new goal point. 
 */
class CloudGoalPublisher {

	private: 
		ros::NodeHandle nh_;

		ros::Publisher cloud_pub_;			// publishes cloud info to /points2
		ros::Publisher goal_pub_;			// publishes goal info to /goal 
		ros::Subscriber success_sub_;		// subcribes to /success from roach_control_pub.cpp

		PointCloud::Ptr cloud_;
		OctreeSearch *octree_search_;

		geometry_msgs::Point goal_pt_;
		std_msgs::Bool success_;

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
		  newt.c_lflag &= ~ICANON;                 	// disable buffering  
		  newt.c_lflag &= ~ECHO; 
		  newt.c_lflag &= ~ISIG;   
		  newt.c_cc[VMIN] = 0;
		  newt.c_cc[VTIME] = 0;
		  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

		  int input = getchar();  					// read character (non-blocking)

		  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);	// restore old settings
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
		CloudGoalPublisher(ros::NodeHandle &nh, int width, float resolution){
			nh_ = nh;
			cloud_pub_ = nh_.advertise<PointCloud>("points2", 1);
			goal_pub_ = nh_.advertise<geometry_msgs::Point>("goal", 1);
			success_sub_ = nh_.subscribe<std_msgs::Bool>("success", 1000, &CloudGoalPublisher::setSuccess, this);

			PointCloud::Ptr cloud(new PointCloud); 	
			cloud_ = cloud;															
			cloud_->header.frame_id = "usb_cam";
			cloud_->height = 1;
			cloud_->width = width; 
			cloud_->points.resize(cloud_->width * cloud_->height);

			octree_search_ = new OctreeSearch(resolution);
			octree_search_->setInputCloud(cloud_);

			// a point with (-100,-100,-100) specifies no goal
			goal_pt_.x = -100; goal_pt_.y = -100; goal_pt_.z = -100;
		}

		/* Callback function used to set if roach has reached the goal point.
		 */
		void setSuccess(const std_msgs::Bool::ConstPtr& msg){
			success_.data = msg->data;
		}

		/* Initially, set goal to be point forward the length of 1 roach body in its current pose.
		 */
		void setStartGoal(string ar_marker){
			cout << "Publishing initial goal..." << endl;
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(5.0));
			  cout << "In setStartGoal(): Looking up tf from usb_cam to " << ar_marker << "...\n";
			  transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}
			pcl::PointXYZ curr_pt(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
			goal_pt_.x = curr_pt.x + ROACH_W;
			goal_pt_.y = curr_pt.y + ROACH_H;
			goal_pt_.z = curr_pt.z;
			goal_pub_.publish(goal_pt_);
			cout << "Set goal initial goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";
		}

		/* Given knowledge about the current point cloud, octree, and location of roach
		 * updates the goal point to the least point-dense location in scene. 
		 */
		void updateGoal(pcl::PointXYZ curr_pt){
			//TODO: WARNING THIS CODE IS NOT COMPLETE, HASN'T BEEN RUN AND ISN'T CORRECT!
			octree_search_->setResolution(max(ROACH_W,ROACH_H));
			double min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;
			octree_search_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
			double bound_width = abs(max_x - min_x);
			double bound_length = abs(max_y - min_y);
			
			int min_pts = INT_MAX;
			Eigen::Vector3f min;
			Eigen::Vector3f min;
			// subdivide bounding box of octree into roach-sized grid
			// search through each grid box for the min
			for(int i = min_x; i < max_x; i += ROACH_W){
				for(int j = min_y; j < max_y; j += ROACH_H){						
					Eigen::Vector3f min_pt(i,j,min_z);
					Eigen::Vector3f max_pt(i+ROACH_W,j+ROACH_H,min_z);
					vector<int> k_indices;
					// search for points within rectangular search area
					octree_search_->boxSearch(min_pt, max_pt, k_indices);
					if(k_indices.size() < min_pts){
						min = min_pt;
						max = max_pt;
						max_pts = k_indices.size();
					}
				}
			}
			// get center of region with the least points
			int w = abs(max[0] - min[0]);				//TODO THIS IS NOT NECESSARILY CORRECT SYNTAX			
			int h = abs(max[1] - min[1]);
			int mid_w = w/2;
			int mid_h = h/2;
			
			// set goal point to be the center of the region with least points
			goal_pt_.x = min[0] + mid_w; 
			goal_pt_.y = max[0] + mid_h;
			goal_pt_.z = min_z;
			cout << "In updateGoal(): New goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";			
		}

		/* Runs point cloud publishing and publishes navigation goals for VelociRoACH
		 *
		 * NOTE: roach_control_pub's exploration method needs to run at lower Hz then 
		 * the cloud_goal_publisher for correct synchronization. 
		 */
		void run(std::string pcd_filename){
			tf::TransformListener transform_listener;
			string ar_marker = "ar_marker_1"; 

			int input = 'c';
			int num_pts = 0;
			int maxCloudSize = cloud_->width*cloud_->height;

			cout << "Press 'q' to quit data collection and save point cloud.\n";

			ros::Rate loop_rate(3);

			/*************** SET INITIAL GOAL FOR VELOCIROACH **************/
			this->setStartGoal(ar_marker);
			/***************************************************************/

			while(nh_.ok() && num_pts < maxCloudSize && input != 'q'){
				input = getch();   // check if user pressed q to quit data collection
				if (input == 'q'){
					break;
				}

				/*********** GET TF FROM USB_CAM TO AR_MARKER ***********/
				tf::StampedTransform transform;
				try{
				  ros::Time now = ros::Time::now();
				  transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(5.0));
				  cout << "In run(): Looking up tf from usb_cam to " << ar_marker << "...\n";
				  transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  ros::Duration(10.0).sleep();
				}
				/*******************************************************/

				/*************** GET GOAL FOR VELOCIROACH **************/
				if(success_.data){ // if roach reached goal
					cout << "CloudGoal: Roach reached goal --> setting new goal..." << endl;
					pcl::PointXYZ curr_pt(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
					if(goal_pt_.x == -100 && goal_pt_.y == -100 && goal_pt_.z == -100){ 
						// if have no goal set goal (or first pass through)
						// set goal to be point forward the length of 1 roach body
						goal_pt_.x = curr_pt.x + ROACH_W;
						goal_pt_.y = curr_pt.y + ROACH_H;
						goal_pt_.z = curr_pt.z;
					}else{
						// query octree for goal point based on density
						this->updateGoal(curr_pt);
					}
					goal_pub_.publish(goal_pt_);
				}else{
					cout << "CloudGoal: Still on old goal: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";
					goal_pub_.publish(goal_pt_);
				}
				/*******************************************************/

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

						num_pts+=1;
						y += 0.02;
					}
					x -= 0.02;
				}
				cloud_->header.stamp = ros::Time::now().toNSec();
				cloud_pub_.publish(cloud_);
				/*********************************************************/

				ros::spinOnce();
				loop_rate.sleep();
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
  ros::init (argc, argv, "cloud_goals");
  ros::NodeHandle nh;

  // Sanity check
  if(argc != 4){
	ROS_ERROR("Not enough arguments! Usage example: cloud_goal_pub test_pcd.pcd 0.01 15000");
	ros::shutdown();
  }

  string pcd_filename = argv[1];
  float resolution = strtof(argv[2], NULL);  
  int width = atoi(argv[3]); 

  cout << "PCD Output Filename: " << pcd_filename << endl;
  cout << "Cloud Size: " << width << endl;
  cout << "Octree Resolution: " << resolution << endl;

  CloudGoalPublisher driver(nh, width, resolution);
  driver.run(pcd_filename);
}


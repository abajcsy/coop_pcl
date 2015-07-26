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

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <coop_pcl/exploration_info.h>

// VelociRoACH measurements in meters
#define ROACH_W 0.04
#define ROACH_H 0.10
#define BOUND_BOX_SZ 0.7

#define GOAL_GRID_W 8
#define GOAL_GRID_H 8

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
		ros::Publisher goal_pub_;			// publishes goal info to /goal_pt 
		ros::Publisher marker_pub_;			// publishes marker representing next goal point to /visualization_marker
		ros::Publisher marker_array_pub_;	// publishes marker array to /visualization_marker_array
		ros::Subscriber success_sub_;		// subcribes to /success from roach_control_pub.cpp

		PointCloud::Ptr cloud_;
		OctreeSearch *octree_search_;

		geometry_msgs::Point goal_pt_;
		std_msgs::Bool success_;

		// represents camera's image divided into GOAL_GRID_H*GOAL_GRID_W rectangular regions
		// stores within it how many times each goal has been assigned
		double goal_grid_[GOAL_GRID_H][GOAL_GRID_W]; 

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
			goal_pub_ = nh_.advertise<geometry_msgs::Point>("goal_pt", 1);
			success_sub_ = nh_.subscribe<std_msgs::Bool>("success", 1000, &CloudGoalPublisher::setSuccess, this);

			marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

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

		/* Publishes marker for RVIZ at location (x,y,z) with (r,g,b) color
		 */
		void publishMarker(geometry_msgs::Point pt, double scaleX, double scaleY, double scaleZ, double r, double g, double b){
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
			marker.scale.x = scaleX;
			marker.scale.y = scaleY;
			marker.scale.z = scaleZ;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			marker_pub_.publish(marker);
		}		

		void publishMarkerArray(vector<geometry_msgs::Point> pts, double sideLen, double r, double g, double b){
			// set up MarkerArray for visualizing in RVIZ
			visualization_msgs::MarkerArray marker_array_msg;
			marker_array_msg.markers.resize(pts.size());
			for(size_t i = 0; i < pts.size(); i++){
				double s = sideLen; 
				double x = pts[i].x;
				double y = pts[i].y;
				double z = pts[i].z;	

				marker_array_msg.markers[i].header.frame_id = "map";
				marker_array_msg.markers[i].header.stamp = ros::Time();
				marker_array_msg.markers[i].ns = "my_namespace";
				marker_array_msg.markers[i].id = i; 
				marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;
				marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

				marker_array_msg.markers[i].pose.position.x = x;
				marker_array_msg.markers[i].pose.position.y = y;
				marker_array_msg.markers[i].pose.position.z = z;
				marker_array_msg.markers[i].pose.orientation.x = 0.0;		
				marker_array_msg.markers[i].pose.orientation.y = 0.0;
				marker_array_msg.markers[i].pose.orientation.z = 0.0;
				marker_array_msg.markers[i].pose.orientation.w = 1.0;

				marker_array_msg.markers[i].scale.x = s;
				marker_array_msg.markers[i].scale.y = s;
				marker_array_msg.markers[i].scale.z = s;

				marker_array_msg.markers[i].color.a = 1.0; 
				if(i == 0){ // color min_bound_box --> pink
					marker_array_msg.markers[i].color.r = 1.0;
					marker_array_msg.markers[i].color.g = 0.0;
					marker_array_msg.markers[i].color.b = 1.0;
				}else if (i == pts.size()-1){ // max_bound_box --> cyan
					marker_array_msg.markers[i].color.r = 0.0;
					marker_array_msg.markers[i].color.g = 1.0;
					marker_array_msg.markers[i].color.b = 1.0;
				}else{
					marker_array_msg.markers[i].color.r = r;
					marker_array_msg.markers[i].color.g = g;
					marker_array_msg.markers[i].color.b = b;
				}
			}
			marker_array_pub_.publish(marker_array_msg);
		}

		/* Initially, set goal to be point forward the length of 1 roach body in its current pose.
		 */
		void setStartGoal(string ar_marker){
			tf::TransformListener transform_listener;
			cout << "In setStartGoal():" << endl;
			cout << "		Publishing initial goal..." << endl;
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform("map", ar_marker, now, ros::Duration(5.0));
			  cout << "		Looking up tf from usb_cam to " << ar_marker << "...\n";
			  transform_listener.lookupTransform("map", ar_marker, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}
			pcl::PointXYZ curr_pt(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
			cloud_pub_.publish(cloud_);(curr_pt.x, curr_pt.y, curr_pt.z, 0.0, 1.0, 0.0);

			cout << "		Current location point: (" << curr_pt.x << ", " << curr_pt.y << ", " << curr_pt.z << ")\n";
			goal_pt_.x = curr_pt.x + 2*ROACH_W;
			goal_pt_.y = curr_pt.y + 2*ROACH_H;
			goal_pt_.z = curr_pt.z;
			goal_pub_.publish(goal_pt_);
			cout << "		Set goal initial goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";

			cout << "		Publishing goal marker..." << endl;
			publishMarker(goal_pt_, 0.05, 0.05, 0.05, 1.0, 0.0, 0.0);
		}

		/* Given knowledge about the current point cloud, octree, and location of roach
		 * updates the goal point to the least point-dense location in scene. 
		 */
		void updateGoal(pcl::PointXYZ curr_pt){
			//TODO: WARNING THIS CODE IS NOT COMPLETE AND ISN'T CORRECT!
			int depth = octree_search_->getTreeDepth();
			cout << "In updateGoal(): " << endl;
			cout << "		Tree Depth: " << depth << endl;

			double min_x = 0.0, min_y = 0.0, min_z = 0.0, max_x = 0.0, max_y = 0.0, max_z = 0.0;
			octree_search_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
			cout << "		Bounding Box Min: (" << min_x << ", " << min_y << ", " << min_z << ")\n";		
			cout << "		Bounding Box Max: (" << max_x << ", " << max_y << ", " << max_z << ")\n";	

			double bound_width = abs(max_x - min_x);
			double bound_length = abs(max_y - min_y);
			double bound_height = abs(max_z - min_z);
			cout << "		Bounding Box Width: " << bound_width << ", Length: " << bound_length << ", Height: " << bound_height << endl;	

			/*** publish marker array for bound box corners ***/
			vector<geometry_msgs::Point> pts;
			geometry_msgs::Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt_mid_x, pt_mid_y, pt_mid_z;
			pt1.x = min_x; pt1.y = min_y; pt1.z = min_z;
			pt2.x = min_x+bound_width; pt2.y = min_y; pt2.z = min_z;
			pt3.x = max_x; pt3.y = max_y; pt3.z = min_z;				
			pt4.x = min_x; pt4.y = min_y+bound_height; pt4.z = min_z;

			pt5.x = min_x; pt5.y = min_y; pt5.z = max_z;
			pt6.x = min_x+bound_width; pt6.y = min_y; pt6.z = max_z;
			pt7.x = min_x; pt7.y = min_y+bound_height; pt7.z = max_z;				
			pt8.x = max_x; pt8.y = max_y; pt8.z = max_z;

			pt_mid_x.x = max_x - (bound_width/2); pt_mid_x.y = max_y; pt_mid_x.z = max_z;
			pt_mid_y.x = max_x; pt_mid_y.y = max_y - (bound_length/2); pt_mid_y.z = max_z;
			pt_mid_z.x = max_x; pt_mid_z.y = max_y; pt_mid_z.z = max_z - (bound_height/2);

			pts.push_back(pt1); pts.push_back(pt2);
			pts.push_back(pt3); pts.push_back(pt4);
			pts.push_back(pt5); pts.push_back(pt6);
			
			pts.push_back(pt_mid_x);			
			pts.push_back(pt_mid_y);
			pts.push_back(pt_mid_z);				

			pts.push_back(pt7); pts.push_back(pt8);
			publishMarkerArray(pts, 0.05, 0.0, 0.0, 1.0);
			/******************************************/

			int min_pts = INT_MAX;
			Eigen::Vector3f max;
			Eigen::Vector3f min;
			// subdivide bounding box of octree into roach-sized grid
			// search through each grid box for the min
			/*for(double col = min_x; col < max_x; col += ROACH_W){ // start in upper left hand corner of bounding box base
				for(double row = max_y; row > min_y; row -= ROACH_H){						
					Eigen::Vector3f min_pt(col, row, min_z);
					Eigen::Vector3f max_pt(col+ROACH_W, row-ROACH_H, min_z);
					vector<int> k_indices;
					// search for points within rectangular search area
					int num_pts_in_box = octree_search_->boxSearch(min_pt, max_pt, k_indices);
					//cout << "		Num points in box region: " << num_pts_in_box << endl;
					if(num_pts_in_box < min_pts){
						min = min_pt;
						max = max_pt;
						min_pts = num_pts_in_box;
						cout << "		Found less dense region: density = " << min_pts << endl;
					}
				}
			}*/	
			double increment = abs(max_x - min_x)/4.0;
			for(double col = min_x; col < max_x; col += increment){ // start in upper left hand corner of bounding box base
				for(double row = max_y; row > min_y; row -= increment){						
					Eigen::Vector3f min_pt(col+increment, row, min_z);
					Eigen::Vector3f max_pt(col, row-increment, BOUND_BOX_SZ);
					vector<int> k_indices;
					// search for points within rectangular search area
					int num_pts_in_box = octree_search_->boxSearch(min_pt, max_pt, k_indices);
					//cout << "		Num points in box region: " << num_pts_in_box << endl;
					if(num_pts_in_box < min_pts){
						min = min_pt;
						max = max_pt;
						min_pts = num_pts_in_box;
						cout << "		Found less dense region: density = " << min_pts << endl;
					}
				}
			}
			
			vector<geometry_msgs::Point> roi_pts;
			geometry_msgs::Point roi_pt1, roi_pt2;
			roi_pt1.x = min[0]; roi_pt1.y = min[1]; roi_pt1.z = min[2];
			roi_pt2.x = max[0]; roi_pt2.y = max[1]; roi_pt2.z = max[2];
			roi_pts.push_back(roi_pt1);
			roi_pts.push_back(roi_pt2);
			publishMarkerArray(roi_pts, 0.05, 1.0, 1.0, 1.0);
			// get center of region with the least points
			double w = abs(max[0] - min[0]);					
			double h = abs(max[1] - min[1]);
			double mid_w = w/2.0;
			double mid_h = h/2.0;
			cout << "		ROI min_pt: (" << min[0] << ", " << min[1] << ", " << min[2] << ")\n";	
			cout << "		ROI max_pt: (" << max[0] << ", " << max[1] << ", " << max[2] << ")\n";		
			cout << "		ROI Width: " << w << ", Height: " << h << endl;
			
			// set goal point to be the center of the region with least points
			goal_pt_.x = min[0] - mid_w; 
			goal_pt_.y = max[1] + mid_h;
			goal_pt_.z = min_z;
			cout << "		New goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";	
			cout << "		Publishing goal marker..." << endl;
					
			publishMarker(goal_pt_, 0.05, 0.05, 0.05, 1.0, 0.0, 0.0);
		}

		/* Computes homography matrix given current coordinates from the initial pose of the AR tag
		 * (which we take to represent the ground plane under the robots feet)
		 */
		cv::Mat computeHomography(string ar_marker){
			tf::TransformListener transform_listener;
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform("map", ar_marker, now, ros::Duration(5.0));
			  cout << "		Looking up tf from usb_cam to " << ar_marker << "...\n";
			  transform_listener.lookupTransform("map", ar_marker, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}
			vector<cv::Point2f> roach_corners, img_corners;
			// get points from point cloud - should be added in clockwise order
			tf::Stamped<tf::Pose> corner1(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.02, -0.05, 0.0)),ros::Time(0), ar_marker),
								corner2(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.02, 0.05, 0.0)),ros::Time(0), ar_marker),
								corner3(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.05, 0.0)),ros::Time(0), ar_marker),
								corner4(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, -0.05, 0.0)),ros::Time(0), ar_marker);
			tf::Stamped<tf::Pose> tf_corner1, tf_corner2, tf_corner3, tf_corner4;
			transform_listener.transformPose("usb_cam", corner1, tf_corner1);
			transform_listener.transformPose("usb_cam", corner2, tf_corner2);
			transform_listener.transformPose("usb_cam", corner3, tf_corner3);
			transform_listener.transformPose("usb_cam", corner4, tf_corner4);
			cv::Point2f pt1(tf_corner1.getOrigin().x(),tf_corner1.getOrigin().y()), 
						pt2(tf_corner2.getOrigin().x(),tf_corner2.getOrigin().y()),
						pt3(tf_corner3.getOrigin().x(),tf_corner3.getOrigin().y()),
						pt4(tf_corner4.getOrigin().x(),tf_corner4.getOrigin().y());
			roach_corners.push_back(pt1); roach_corners.push_back(pt2);
			roach_corners.push_back(pt3); roach_corners.push_back(pt4);
			// get corner points from camera image - also added in clockwise order
			cv::Point2f img_pt1(0.0,480.0), img_pt2(0.0,0.0), img_pt3(640.0,0.0), img_pt4(640.0,480.0);
			img_corners.push_back(img_pt1); img_corners.push_back(img_pt2);
			img_corners.push_back(img_pt3); img_corners.push_back(img_pt4);
		
			cv::Mat H = cv::findHomography(roach_corners, img_corners, 0);	// note: 0 means we use regular method to compute homography matrix using all points
			return H;
		}
		
		void assignGoal(){
			srand(time(NULL));
			double gridRegion = rand() % (GOAL_GRID_W*GOAL_GRID_H)+ 1; // get random number between 1 and GOAL_GRID_W*GOAL_GRID_H
			
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
			cv::Mat H = this->computeHomography(ar_marker);
			/***************************************************************/

			while(nh_.ok() && num_pts < maxCloudSize && input != 'q'){
				input = getch();   // check if user pressed q to quit data collection
				if (input == 'q'){
					break;
				}
				cout << "In run():" << endl;
				/*********** GET TF FROM MAP TO AR_MARKER ***********/
				tf::StampedTransform transform;
				try{
				  ros::Time now = ros::Time::now();
				  transform_listener.waitForTransform("map", ar_marker, now, ros::Duration(5.0));
				  cout << "		Looking up tf from usb_cam to " << ar_marker << "...\n";
				  transform_listener.lookupTransform("map", ar_marker, ros::Time(0), transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  ros::Duration(10.0).sleep();
				}
				/*******************************************************/

				/*************** GET GOAL FOR VELOCIROACH **************/
				if(success_.data){ // if roach reached goal
					cout << "		CloudGoal: Roach reached goal --> setting new goal..." << endl;
					pcl::PointXYZ curr_pt(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
					cloud_pub_.publish(cloud_);(curr_pt.x, curr_pt.y, curr_pt.z, 0.0, 1.0, 0.0);
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
					cout << "		CloudGoal: Still on old goal: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";
					goal_pub_.publish(goal_pt_);
				}
				/*******************************************************/

				/********* PUBLISH POINT CLOUD UNDER VELOCIROACH *********/
				for(double x = 0.05; x >= -0.05; x -= 0.02){
					for(double y = -0.02; y <= 0.02; y += 0.02){
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
						octree_search_->setInputCloud(cloud_);
					 	octree_search_->addPointsFromInputCloud();

						num_pts+=1;
					}
				}
				cout << "		Publishing point cloud.\n";
				cloud_->header.stamp = ros::Time::now().toNSec();
				cloud_pub_.publish(cloud_);
				cout << "		Finished publishing point cloud..." << endl;
				/*********************************************************/
				//cout << "			Current location point: (" << curr_pt.x << ", " << curr_pt.y << ", " << curr_pt.z << ")\n";
				//this->updateGoal(curr_pt);

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

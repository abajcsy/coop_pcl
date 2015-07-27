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
#define CAMERA_IMG_W 640.0
#define CAMERA_IMG_H 480.0

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
		cv::Mat homography_;
		cv::Mat homography_inverse_;

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

		/* Computes homography matrix given current coordinates from the initial pose of the AR tag
		 * (which we take to represent the ground plane under the robots feet)
		 */
		void computeHomography(string ar_marker){
			cout << "In computeHomography():" << endl;
			tf::TransformListener transform_listener;
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform("map", ar_marker, now, ros::Duration(5.0));
			  cout << "		Looking up tf from map to " << ar_marker << "...\n";
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
			transform_listener.transformPose("map", corner1, tf_corner1);
			transform_listener.transformPose("map", corner2, tf_corner2);
			transform_listener.transformPose("map", corner3, tf_corner3);
			transform_listener.transformPose("map", corner4, tf_corner4);
			/*cv::Point2f pt1(tf_corner1.getOrigin().x(),tf_corner1.getOrigin().y()), 
						pt2(tf_corner2.getOrigin().x(),tf_corner2.getOrigin().y()),
						pt3(tf_corner3.getOrigin().x(),tf_corner3.getOrigin().y()),
						pt4(tf_corner4.getOrigin().x(),tf_corner4.getOrigin().y());*/
			cv::Point2f pt1(0.0,0.0), 
						pt2(0.0,0.1),
						pt3(0.04,0.1),
						pt4(0.04,0.0);
			roach_corners.push_back(pt1); roach_corners.push_back(pt2);
			roach_corners.push_back(pt3); roach_corners.push_back(pt4);
			cout << "		Roach corners:" << endl;
			for(int i = 0; i < roach_corners.size(); i++){
				cout << "		(" << roach_corners[i].x << ", " << roach_corners[i].y << ")\n";
			}
			// get corner points from camera image - also added in clockwise order
			cv::Point2f img_pt1(0.0,CAMERA_IMG_H), img_pt2(0.0,0.0), img_pt3(CAMERA_IMG_W,0.0), img_pt4(CAMERA_IMG_W,CAMERA_IMG_H);
			img_corners.push_back(img_pt1); img_corners.push_back(img_pt2);
			img_corners.push_back(img_pt3); img_corners.push_back(img_pt4);
		
			// note: 0 means we use regular method to compute homography matrix using all points
			homography_ = cv::findHomography(roach_corners, img_corners, 0);	
			homography_inverse_ = homography_.inv();	
			cout << "		Printing Homography Matrix:" << endl;						
			for(int i = 0; i < homography_.rows; i++){
				cout << "		";
				for(int j = 0; j < homography_.cols; j++){
					cout << homography_.at<double>(i,j) << " ";
				}
				cout << endl;
			}
			cout << "		Printing Inverse Homography Matrix:" << endl;						
			for(int i = 0; i < homography_inverse_.rows; i++){
				cout << "		";
				for(int j = 0; j < homography_inverse_.cols; j++){
					cout << homography_inverse_.at<double>(i,j) << " ";
				}
				cout << endl;
			}
			//---------publish marker array in rviz for boundary of FOV ----------//
			double ll_data[3] = {0.0, CAMERA_IMG_H, 1.0};
			double ul_data[3] = {0.0, 0.0, 1.0};
			double ur_data[3] = {CAMERA_IMG_W, 0.0, 1.0};
			double lr_data[3] = {CAMERA_IMG_W, CAMERA_IMG_H, 1.0};
			cv::Mat ll = cv::Mat(3,1, CV_64F, ll_data);
			cv::Mat ul = cv::Mat(3,1, CV_64F, ul_data);
			cv::Mat ur = cv::Mat(3,1, CV_64F, ur_data);
			cv::Mat lr = cv::Mat(3,1, CV_64F, lr_data);
			cv::Mat ll_result = homography_inverse_*ll;
			cv::Mat ul_result = homography_inverse_*ul;
			cv::Mat ur_result = homography_inverse_*ur;
			cv::Mat lr_result = homography_inverse_*lr;
			double ll_x = ll_result.at<double>(0,0)/ll_result.at<double>(2,0); double ll_y = ll_result.at<double>(1,0)/ll_result.at<double>(2,0);
			double ul_x = ul_result.at<double>(0,0)/ul_result.at<double>(2,0); double ul_y = ul_result.at<double>(1,0)/ul_result.at<double>(2,0);
			double ur_x = ur_result.at<double>(0,0)/ur_result.at<double>(2,0); double ur_y = ur_result.at<double>(1,0)/ur_result.at<double>(2,0);
			double lr_x = lr_result.at<double>(0,0)/lr_result.at<double>(2,0); double lr_y = lr_result.at<double>(1,0)/lr_result.at<double>(2,0);
			vector<geometry_msgs::Point> pts;
			geometry_msgs::Point p1, p2, p3, p4;
			p1.x = ll_x; p1.y = ll_y; p1.z = 0.0;
			p2.x = ul_x; p2.y = ul_y; p2.z = 0.0;
			p3.x = ur_x; p3.y = ur_y; p3.z = 0.0;
			p4.x = lr_x; p4.y = lr_y; p4.z = 0.0;
			cout << "		ll: (" << ll_x << ", " << ll_y << ")\n";
			cout << "		ul: (" << ul_x << ", " << ul_y << ")\n";
			cout << "		ur: (" << ur_x << ", " << ur_y << ")\n";
			cout << "		lr: (" << lr_x << ", " << lr_y << ")\n";
			pts.push_back(p1);
			pts.push_back(p2);
			pts.push_back(p3);
			pts.push_back(p4);
			publishMarkerArray(pts, 0.05, 0.0, 0.0, 1.0);
		}
		
		/* Assigns the next goal location by dividing the camera image plane into GOAL_GRID_H*GOAL_GRID_W
		 * grid rectangles, each with 1/GOAL_GRID_H*GOAL_GRID_W probability of being selected.
		 * Uses homography matrix to go from the selected region in the camera image to real-life coordinates
		 * in the exploration plane. 
		 */
		void assignGoal(){
			cout << "In assignGoal():" << endl;
			srand(time(NULL));
			// get random number between 0 and GOAL_GRID_H-1
			int row = rand() % (GOAL_GRID_H); 
			int col = rand() % (GOAL_GRID_W);
			cout << "		row = " << row << ", col = " << col << endl;
			// store that we have visited this location before
			goal_grid_[row][col] += 1.0; 
			// upper left corner of ROI
			double rect_h = CAMERA_IMG_H/GOAL_GRID_H; double rect_w = CAMERA_IMG_W/GOAL_GRID_W;
			double ul_corner_x = col*rect_h; double ul_corner_y = row*rect_w; 
			// store center of ROI 
			cv::Mat centroid(3,1,CV_64F);
			centroid.at<double>(0,0) = ul_corner_x+(rect_w/2.0);
			centroid.at<double>(1,0) = ul_corner_y+(rect_h/2.0);
			centroid.at<double>(2,0) = 1;
			cout << "		centroid: (" << centroid.at<double>(0,0) << ", " << centroid.at<double>(1,0) << ")\n";
			// get real-world coordinates using homography inverse
			cv::Mat result = homography_inverse_*centroid;
			double x = result.at<double>(0,0)/result.at<double>(2,0);
			double y = result.at<double>(1,0)/result.at<double>(2,0);
			// set goal point
			goal_pt_.x = x;
			goal_pt_.y = y;
			goal_pt_.z = 0.0;												//TODO: WHAT SHOULD THE Z-VALUE BE??? (treating 3d problem as 2d problem)
			cout << "		New goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";	
			cout << "		Publishing goal marker..." << endl;
			// publish goal marker for rviz visualization
			publishMarker(goal_pt_, 0.05, 0.05, 0.05, 1.0, 0.0, 0.0);
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
			this->computeHomography(ar_marker);
			this->assignGoal();
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
				  cout << "		Looking up tf from map to " << ar_marker << "...\n";
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
					if(goal_pt_.x == -100 && goal_pt_.y == -100 && goal_pt_.z == -100){ 
						// if have no goal set goal (or first pass through)
						// set goal to be point forward the length of 1 roach body
						goal_pt_.x = curr_pt.x + ROACH_W;
						goal_pt_.y = curr_pt.y + ROACH_H;
						goal_pt_.z = curr_pt.z;
					}else{
						this->assignGoal();
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

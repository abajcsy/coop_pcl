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
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>

#include <stdio.h>
#include <termios.h>
#include <math.h> 
#include <limits.h>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <coop_pcl/exploration_info.h>

// VelociRoACH measurements in meters
#define ROACH_W 0.07
#define ROACH_H 0.11

// divide field of view (FOV) of camera into 8x8 grid
#define GOAL_GRID_W 8
#define GOAL_GRID_H 8

// store pixel dimensions of camera
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
		ros::Subscriber camera_info_sub_;	// subscribes to /usb_cam/camera_info to get camera projection matrix

		tf::TransformListener transform_listener;

		PointCloud::Ptr cloud_;
		OctreeSearch *octree_search_;

		geometry_msgs::Point goal_pt_;		// next goal location for roach to move to
		std_msgs::Bool success_;			// stores if the roach was able to reach goal location

		cv::Mat homography_;				// homography between ground coords and camera pixels
		cv::Mat homography_inverse_;		
		cv::Mat P_;							// camera projection matrix (comes from camera_info topic)

		// stores the lower-left, upper-left, upper-right, lower-right corners of the homography plane based on camera dimensions
		vector<geometry_msgs::Point> camera_homography_pts_;	

		// represents camera's image divided into GOAL_GRID_H*GOAL_GRID_W rectangular regions
		// stores within it the probability of landing on that grid location
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
			camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("usb_cam/camera_info", 1000, &CloudGoalPublisher::setP, this);

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

			// initially, every grid location has equal chance of being selected
			double prob = 1.0/(GOAL_GRID_H*GOAL_GRID_W);
			cout << "In constructor():" << endl;
			cout << "prob: " << prob;
			for(int row = 0; row < GOAL_GRID_H; row++){
				for(int col = 0; col < GOAL_GRID_W; col++){
					goal_grid_[row][col] = prob;
					cout << goal_grid_[row][col] << " ";
				}
				cout << endl;			
			}
		}

		/* Callback function used to set if roach has reached the goal point.
		 */
		void setSuccess(const std_msgs::Bool::ConstPtr& msg){
			success_.data = msg->data;
		}

		/* Callback function used to set Projection/camera matrix from CameraInfo
		 */
		void setP(const sensor_msgs::CameraInfo::ConstPtr& msg){
			P_ = cv::Mat(3,4, CV_64FC1, double(0));
			int idx = 0;
			for(int i = 0; i < P_.rows; i++){
				for(int j = 0; j < P_.cols; j++){
					P_.at<double>(i,j) = msg->P[idx]; // get 3x4 row-major matrix as 1D array (have to reshape)
					idx++;
				}	
			}
		}

		/* Publishes marker for RVIZ at location (x,y,z) with (r,g,b) color
		 */
		void publishMarker(geometry_msgs::Point pt, double scaleX, double scaleY, double scaleZ, double r, double g, double b){
			visualization_msgs::Marker marker;
			marker.header.frame_id = "usb_cam";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
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

		/* Publishes group of markers for RVIZ with cubes of sidelength and (r,g,b) color
		 */
		void publishMarkerArray(vector<geometry_msgs::Point> pts, double sideLen, double r, double g, double b){
			// set up MarkerArray for visualizing in RVIZ
			visualization_msgs::MarkerArray marker_array_msg;
			marker_array_msg.markers.resize(pts.size());
			for(size_t i = 0; i < pts.size(); i++){
				double s = sideLen; 
				double x = pts[i].x;
				double y = pts[i].y;
				double z = pts[i].z;	

				marker_array_msg.markers[i].header.frame_id = "usb_cam";
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
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform(ar_marker, "usb_cam", now, ros::Duration(5.0));
			  cout << "		Looking up tf from " << ar_marker << " to usb_cam...\n";
			  transform_listener.lookupTransform(ar_marker, "usb_cam", ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}
			vector<cv::Point2f> roach_corners, img_corners;
			/*************** get real world measurements of roach corners in counter clockwise order ************/
			cv::Point2f pt1(0.0,0.0), 
						pt2(0.0,ROACH_H),
						pt3(ROACH_W,ROACH_H),
						pt4(ROACH_W,0.0);
			roach_corners.push_back(pt1); roach_corners.push_back(pt2);
			roach_corners.push_back(pt3); roach_corners.push_back(pt4);
			/****************************************************************************************************/

			/*************************** get roach corners in the usb_cam frame *********************************/
			/* (assumption is that the ar_marker is lying on its back with z pointing upwards, x to the right and y forward)  */
			tf::Stamped<tf::Pose> ll_corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.055,-0.035, -0.08)),ros::Time(0), ar_marker),
					ul_corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.055, -0.035, -0.08)),ros::Time(0), ar_marker),
					ur_corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.055, 0.035, -0.08)),ros::Time(0), ar_marker),
					lr_corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.055, 0.035, -0.08)),ros::Time(0), ar_marker);
			tf::Stamped<tf::Pose> tf_ll_corner, tf_ul_corner, tf_ur_corner, tf_lr_corner;
			transform_listener.transformPose("usb_cam", ll_corner, tf_ll_corner);
			transform_listener.transformPose("usb_cam", ul_corner, tf_ul_corner);
			transform_listener.transformPose("usb_cam", ur_corner, tf_ur_corner);
			transform_listener.transformPose("usb_cam", lr_corner, tf_lr_corner);
			// convert to stupid opencv matrix...
			cv::Mat ll_xyz1 = (cv::Mat_<double>(4,1) << tf_ll_corner.getOrigin().x(), tf_ll_corner.getOrigin().y(), tf_ll_corner.getOrigin().z(), 1.0);
			cv::Mat ul_xyz1 = (cv::Mat_<double>(4,1) << tf_ul_corner.getOrigin().x(), tf_ul_corner.getOrigin().y(), tf_ul_corner.getOrigin().z(), 1.0);
			cv::Mat ur_xyz1 = (cv::Mat_<double>(4,1) << tf_ur_corner.getOrigin().x(), tf_ur_corner.getOrigin().y(), tf_ur_corner.getOrigin().z(), 1.0);
			cv::Mat lr_xyz1 = (cv::Mat_<double>(4,1) << tf_lr_corner.getOrigin().x(), tf_lr_corner.getOrigin().y(), tf_lr_corner.getOrigin().z(), 1.0);
			// make sure to wait until P_ is initialized with message from CameraInfo
			while(P_.rows == 0 && P_.cols == 0){	
				ros::spinOnce();
			}
			// get pixel coordinates of roach corners using projection/camera matrix P_
			cv::Mat trans_ll_xyz1 = P_*ll_xyz1;
			cv::Mat trans_ul_xyz1 = P_*ul_xyz1;
			cv::Mat trans_ur_xyz1 = P_*ur_xyz1;
			cv::Mat trans_lr_xyz1 = P_*lr_xyz1;
			cv::Point2f img_pt1(trans_ll_xyz1.at<double>(0,0)/trans_ll_xyz1.at<double>(2,0), trans_ll_xyz1.at<double>(1,0)/trans_ll_xyz1.at<double>(2,0)), 
						img_pt2(trans_ul_xyz1.at<double>(0,0)/trans_ul_xyz1.at<double>(2,0), trans_ul_xyz1.at<double>(1,0)/trans_ul_xyz1.at<double>(2,0)),
						img_pt3(trans_ur_xyz1.at<double>(0,0)/trans_ur_xyz1.at<double>(2,0), trans_ur_xyz1.at<double>(1,0)/trans_ur_xyz1.at<double>(2,0)),
						img_pt4(trans_lr_xyz1.at<double>(0,0)/trans_lr_xyz1.at<double>(2,0), trans_lr_xyz1.at<double>(1,0)/trans_lr_xyz1.at<double>(2,0));
			img_corners.push_back(img_pt4); img_corners.push_back(img_pt1);
			img_corners.push_back(img_pt2); img_corners.push_back(img_pt3);

			cout << "		Pixel-coords Roach corners:" << endl;
			for(int i = 0; i < img_corners.size(); i++){
				cout << "		(" << img_corners[i].x << ", " << img_corners[i].y << ")\n";
			}
			cout << "		Real-life Roach corners:" << endl;
			for(int i = 0; i < roach_corners.size(); i++){
				cout << "		(" << roach_corners[i].x << ", " << roach_corners[i].y << ")\n";
			}
			/****************************************************************************************************/
		
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

			/************************* publish marker array in rviz for boundary of FOV *************************/
			// pixel coordinates of usb_cam image
			double ll_data[3] = {0.0, CAMERA_IMG_H, 1.0};
			double ul_data[3] = {0.0, 0.0, 1.0};
			double ur_data[3] = {CAMERA_IMG_W, 0.0, 1.0};
			double lr_data[3] = {CAMERA_IMG_W, CAMERA_IMG_H, 1.0};
			// convert to stupid opencv matrix
			cv::Mat ll = cv::Mat(3,1, CV_64F, ll_data);
			cv::Mat ul = cv::Mat(3,1, CV_64F, ul_data);
			cv::Mat ur = cv::Mat(3,1, CV_64F, ur_data);
			cv::Mat lr = cv::Mat(3,1, CV_64F, lr_data);
			// apply H^(-1) to go from pixel to real-world coordinates
			cv::Mat ll_result = homography_inverse_*ll;
			cv::Mat ul_result = homography_inverse_*ul;
			cv::Mat ur_result = homography_inverse_*ur;
			cv::Mat lr_result = homography_inverse_*lr;
			// get (x,y) coordinates in real world for each corner of camera frame
			double ll_x = ll_result.at<double>(0,0)/ll_result.at<double>(2,0); double ll_y = ll_result.at<double>(1,0)/ll_result.at<double>(2,0); 
			double ul_x = ul_result.at<double>(0,0)/ul_result.at<double>(2,0); double ul_y = ul_result.at<double>(1,0)/ul_result.at<double>(2,0);
			double ur_x = ur_result.at<double>(0,0)/ur_result.at<double>(2,0); double ur_y = ur_result.at<double>(1,0)/ur_result.at<double>(2,0);
			double lr_x = lr_result.at<double>(0,0)/lr_result.at<double>(2,0); double lr_y = lr_result.at<double>(1,0)/lr_result.at<double>(2,0);
			cout << "		ll corner of camera in homography: (" << ll_x << ", " << ll_y << ")\n";
			cout << "		ul corner of camera in homography: (" << ul_x << ", " << ul_y << ")\n";
			cout << "		ur corner of camera in homography: (" << ur_x << ", " << ur_y << ")\n";
			cout << "		lr corner of camera in homography: (" << lr_x << ", " << lr_y << ")\n";
			// get z value by transforming from homography plane points to usb_cam
			tf::Stamped<tf::Pose> ll_cam(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(ll_x, ll_y, 0.0)),ros::Time(0), ar_marker),
			ul_cam(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(ul_x, ul_y, 0.0)),ros::Time(0), ar_marker),
			ur_cam(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(ur_x, ur_y, 0.0)),ros::Time(0), ar_marker),
			lr_cam(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(lr_x, lr_y, 0.0)),ros::Time(0), ar_marker);
			transform_listener.transformPose("usb_cam", ll_cam, tf_ll_corner);
			transform_listener.transformPose("usb_cam", ul_cam, tf_ul_corner);
			transform_listener.transformPose("usb_cam", ur_cam, tf_ur_corner);
			transform_listener.transformPose("usb_cam", lr_cam, tf_lr_corner);
			// convert to vector<geometry_msgs::Point> for RVIZ 
			vector<geometry_msgs::Point> pts;
			geometry_msgs::Point p1, p2, p3, p4;
			p1.x = tf_ll_corner.getOrigin().x(); p1.y = tf_ll_corner.getOrigin().y(); p1.z = tf_ll_corner.getOrigin().z();		
			p2.x = tf_ul_corner.getOrigin().x(); p2.y = tf_ul_corner.getOrigin().y(); p2.z = tf_ul_corner.getOrigin().z();
			p3.x = tf_ur_corner.getOrigin().x(); p3.y = tf_ur_corner.getOrigin().y(); p3.z = tf_ur_corner.getOrigin().z();
			p4.x = tf_lr_corner.getOrigin().x(); p4.y = tf_lr_corner.getOrigin().y(); p4.z = tf_lr_corner.getOrigin().z();
			cout << "		ll corner of camera in usb_cam: (" << p1.x << ", " << p1.y << ", " << p1.z << ")\n";
			cout << "		ul corner of camera in usb_cam: (" << p2.x << ", " << p2.y << ", " << p2.z << ")\n";
			cout << "		ur corner of camera in usb_cam: (" << p3.x << ", " << p3.y << ", " << p3.z << ")\n";
			cout << "		lr corner of camera in usb_cam: (" << p4.x << ", " << p4.y << ", " << p4.z << ")\n";
			pts.push_back(p1); pts.push_back(p2);
			pts.push_back(p3); pts.push_back(p4);
			// store points for later
			camera_homography_pts_ = pts;
			publishMarkerArray(pts, 0.02, 0.0, 0.0, 1.0);
			/***************************************************************************************************/
		}
		
		/* Assigns the next goal location by dividing the camera image plane into GOAL_GRID_H*GOAL_GRID_W
		 * grid rectangles, each with 1/GOAL_GRID_H*GOAL_GRID_W probability of being selected.
		 * Uses homography matrix to go from the selected region in the camera image to real-life coordinates
		 * in the exploration plane. 
		 */
		void assignGoal(string ar_marker){
			cout << "In assignGoal():" << endl;
			double pdf[GOAL_GRID_W*GOAL_GRID_H];
			double prev = 0;
			int i = 0;
			/*** print goal_grid_ ***/
			cout << "goal grid: " << endl;
			for(int row = 0 ; row < GOAL_GRID_H; row++){
				for(int col = 0; col < GOAL_GRID_W; col++){
					pdf[i] = prev+goal_grid_[row][col];
					prev += goal_grid_[row][col];
					i++;
					cout << goal_grid_[row][col] << " ";
				}
				cout << endl;
			}
			cout << "pdf: " << endl;
			for(int i = 0 ; i < GOAL_GRID_H*GOAL_GRID_W; i++){
				cout  << pdf[i] << " ";
			}
			cout << endl;
			srand(time(NULL));
			// get random number between 0.0 and 1.0
			double randNum = randDouble(0.0, 1.0);
			cout << "		randNum: " << randNum << endl;
			int gridCell = 0; 
			// find which grid cell was chosen
			for(int i = 0; i < GOAL_GRID_H*GOAL_GRID_W; i++){
				if(i == 0 && randNum <= pdf[i]){
					gridCell = i+1;
					break;
				}else if(pdf[i-1] < randNum && randNum <= pdf[i]){
					gridCell = i+1;
					break;
				}else if(i == GOAL_GRID_H*GOAL_GRID_W-1 && randNum > pdf[i]){
					gridCell = i+1;
					break;
				}
			}
			cout << "		gridCell: " << gridCell << endl;
			// convert grid cell to row, col for indexing into image plane
			int row = 0, col = 0;
			int tmpCell = gridCell;
			while(tmpCell > 8){
				tmpCell -= GOAL_GRID_W;
				row++;
			}
			col = tmpCell-1;
			cout << "		row = " << row << ", col = " << col << endl;
			// upper left corner of ROI
			double rect_h = CAMERA_IMG_H/GOAL_GRID_H; double rect_w = CAMERA_IMG_W/GOAL_GRID_W;
			double ul_corner_x = col*rect_h; double ul_corner_y = row*rect_w; 
			// store center of ROI 
			cv::Mat centroid(3,1,CV_64F);
			centroid.at<double>(0,0) = ul_corner_x+(rect_w/2.0);
			centroid.at<double>(1,0) = ul_corner_y+(rect_h/2.0);
			centroid.at<double>(2,0) = 1;
			cout << "		centroid: (" << centroid.at<double>(0,0) << ", " << centroid.at<double>(1,0) << ")\n";
			// get real-world (x,y) coordinates using homography inverse
			cv::Mat result = homography_inverse_*centroid;
			double x = result.at<double>(0,0)/result.at<double>(2,0);
			double y = result.at<double>(1,0)/result.at<double>(2,0);
			// get z value by transforming from homography plane points to usb_cam
			tf::Stamped<tf::Pose> cam_pt(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, 0.0)),ros::Time(0), ar_marker);
			tf::Stamped<tf::Pose> tf_cam_pt;
			transform_listener.transformPose("usb_cam", cam_pt, tf_cam_pt);
			// set goal point
			goal_pt_.x = tf_cam_pt.getOrigin().x();
			goal_pt_.y = tf_cam_pt.getOrigin().y();
			goal_pt_.z = tf_cam_pt.getOrigin().z();												
			cout << "		New goal point: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";	
			cout << "		Publishing goal marker..." << endl;
			// publish goal marker for rviz visualization
			publishMarker(goal_pt_, 0.04, 0.04, 0.04, 0.0, 1.0, 0.0);	
		}

		/* Runs point cloud publishing and publishes navigation goals for VelociRoACH
		 *
		 * NOTE: roach_control_pub's exploration method needs to run at lower Hz then 
		 * the cloud_goal_publisher for correct synchronization. 
		 */
		void run(std::string pcd_filename){
			string ar_marker = "ar_marker_0_bundle"; 

			int input = 'c';
			int num_pts = 0;
			int maxCloudSize = cloud_->width*cloud_->height;

			cout << "Press 'q' to quit data collection and save point cloud.\n";

			ros::Rate loop_rate(3);

			/*************** SET INITIAL GOAL FOR VELOCIROACH **************/
			this->computeHomography(ar_marker);
			this->assignGoal(ar_marker);
			/***************************************************************/

			while(nh_.ok() && num_pts < maxCloudSize && input != 'q'){
				input = getch();   // check if user pressed q to quit data collection
				if (input == 'q'){
					break;
				}
				cout << "In run():" << endl;

				/*************** GET GOAL FOR VELOCIROACH **************/
				if(success_.data && goal_pt_.x == -100 && goal_pt_.y == -100 && goal_pt_.x == -100){ // if roach reached goal
					cout << "		CloudGoal: Roach reached goal --> setting new goal..." << endl;
					this->assignGoal(ar_marker);
					goal_pub_.publish(goal_pt_);
				}else{
					//cout << "		CloudGoal: Still on old goal: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n";
					goal_pub_.publish(goal_pt_);
				}
				/*******************************************************/

				/********* PUBLISH POINT CLOUD UNDER VELOCIROACH *********/
				for(double x = -0.055; x <= 0.055; x += 0.02){
					for(double y = -0.035; y <= 0.035; y += 0.02){
						// get stamped pose wrt ar_marker (note: -0.08 is the z-offset for the base of the roach wrt the top ar marker)
						tf::Stamped<tf::Pose> corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, -0.08)),ros::Time(0), ar_marker);
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
				//cout << "		Publishing point cloud.\n";
				cloud_->header.stamp = ros::Time::now().toNSec();
				cloud_pub_.publish(cloud_);
				//cout << "		Finished publishing point cloud..." << endl;
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

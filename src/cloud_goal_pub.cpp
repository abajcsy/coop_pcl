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
#include <tf/transform_broadcaster.h>

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
#define GOAL_GRID_W 6
#define GOAL_GRID_H 6

// store pixel dimensions of camera
#define CAMERA_IMG_W 640
#define CAMERA_IMG_H 480

#define PI 3.14159265

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
		ros::Publisher hom_cloud_pub_;		// publishes homography plane point cloud info to /hom_points2
		ros::Publisher goal_pub_;			// publishes goal info to /goal_pt 
		ros::Publisher marker_pub_;			// publishes marker representing next goal point to /visualization_marker
		ros::Publisher marker_array_pub_;	// publishes marker array to /visualization_marker_array
		ros::Subscriber success_sub_;		// subcribes to /success from roach_control_pub.cpp
		ros::Subscriber fail_sub_;			// subcribes to /fail from roach_control_pub.cpp
		ros::Subscriber camera_info_sub_;	// subscribes to /usb_cam/camera_info to get camera projection matrix

		tf::TransformListener transform_listener;
		tf::TransformBroadcaster homography_broadcaster_;

		PointCloud::Ptr cloud_;
		PointCloud::Ptr hom_cloud_;
		OctreeSearch *octree_search_;

		int num_pts; 		// num pts in point cloud
		int hom_num_pts;	// num pts in homography plane point cloud

		geometry_msgs::Point goal_pt_;		// next goal location for roach to move to
		std_msgs::Bool success_;			// stores if the roach was able to reach goal location
		std_msgs::Bool fail_flag_;			// stores if the roach FAILED to reach goal location after N attempts

		cv::Mat homography_;				// homography between ground coords and camera pixels
		cv::Mat homography_inverse_;		
		cv::Mat P_;							// camera projection matrix (comes from camera_info topic)
	
		// represents camera's image divided into GOAL_GRID_H*GOAL_GRID_W rectangular regions
		// stores within it the probability of landing on that grid location
		double goal_grid_[GOAL_GRID_H][GOAL_GRID_W]; 
		int goal_row_, goal_col_;

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
	     * Considering the scale of the velociroach map (where the velociroach is 0.07 x 0.11 m) we want very fine resolution
		 */
		CloudGoalPublisher(ros::NodeHandle &nh, int width, float resolution){
			nh_ = nh;
			cloud_pub_ = nh_.advertise<PointCloud>("points2", 1);
			hom_cloud_pub_ = nh_.advertise<PointCloud>("hom_points2", 1);
			goal_pub_ = nh_.advertise<geometry_msgs::Point>("goal_pt", 1);
			success_sub_ = nh_.subscribe<std_msgs::Bool>("success", 1000, &CloudGoalPublisher::setSuccess, this);
			fail_sub_ = nh_.subscribe<std_msgs::Bool>("fail", 1000, &CloudGoalPublisher::setFail, this);
			camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("usb_cam/camera_info", 1000, &CloudGoalPublisher::setP, this);

			marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

			PointCloud::Ptr cloud(new PointCloud); 	
			cloud_ = cloud;															
			cloud_->header.frame_id = "usb_cam";
			cloud_->height = 1;
			cloud_->width = width; 
			cloud_->points.resize(cloud_->width * cloud_->height);
			num_pts = 0;

			octree_search_ = new OctreeSearch(resolution);
			octree_search_->setInputCloud(cloud_);

			PointCloud::Ptr hom_cloud(new PointCloud); 	
			hom_cloud_ = hom_cloud;															
			hom_cloud_->header.frame_id = "homography_plane";
			hom_cloud_->height = 1;
			hom_cloud_->width = width; 
			hom_cloud_->points.resize(hom_cloud_->width * hom_cloud_->height);
			hom_num_pts = 0;

			success_.data = false;

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

		/* Callback function used to set if roach has FAILED to reached the goal point.
		 */
		void setFail(const std_msgs::Bool::ConstPtr& msg){
			fail_flag_.data = msg->data;
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

		/* Publishes goal marker for RVIZ at location (x,y,z) with (r,g,b) color
		 */
		void publishGoalMarker(geometry_msgs::Point pt, double scaleX, double scaleY, double scaleZ, double r, double g, double b){
			visualization_msgs::Marker marker;
			marker.header.frame_id = "homography_plane";
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
				}else if(i == pts.size()-1){ // max_bound_box --> cyan
					marker_array_msg.markers[i].color.r = 0.0;
					marker_array_msg.markers[i].color.g = 1.0;
					marker_array_msg.markers[i].color.b = 1.0;
				}else if(i == 1){
					marker_array_msg.markers[i].color.r = 1.0;
					marker_array_msg.markers[i].color.g = 0.0;
					marker_array_msg.markers[i].color.b = 0.0;
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
			  transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(5.0));
			  cout << "		Looking up tf from " << ar_marker << " to usb_cam...\n";
			  transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}
			vector<cv::Point2f> roach_corners, img_corners;
			/*************** get real world measurements of roach corners in counter clockwise order ************/
			cv::Point2f pt1(0.0,0.0), 
						pt2(ROACH_H,0.0),
						pt3(ROACH_H,-ROACH_W),
						pt4(0.0,-ROACH_W);
			roach_corners.push_back(pt1); roach_corners.push_back(pt2);
			roach_corners.push_back(pt3); roach_corners.push_back(pt4);
			cout << "		Real-life Roach corners:" << endl;
			for(int i = 0; i < roach_corners.size(); i++){
				cout << "		(" << roach_corners[i].x << ", " << roach_corners[i].y << ")\n";
			}
			/****************************************************************************************************/

			/*************************** get roach corners in the usb_cam frame *********************************/
			/* (assumption is that the ar_marker is lying on its back with z pointing upwards, x forward and y pointing left)  */
			tf::Vector3 ll(-0.055, 0.035, -0.08), ul(0.055, 0.035, -0.08), ur(0.055, -0.035, -0.08), lr(-0.055, -0.035, -0.08);
			vector<geometry_msgs::Point> roach_pts;
			vector<tf::Vector3> orig_corners;
			orig_corners.push_back(ll);
			orig_corners.push_back(ul);
			orig_corners.push_back(ur);
			orig_corners.push_back(lr);
			for(int i = 0; i < orig_corners.size(); i++){
				tf::Vector3 pt = orig_corners[i];
				tf::Stamped<tf::Pose> pt_pose(tf::Pose(tf::Quaternion(0, 0, 0, 1), pt), ros::Time(0), ar_marker);
				tf::Stamped<tf::Pose> tf_pt_pose;
				// transform from ar_marker to usb_cam frame
				transform_listener.transformPose("usb_cam", pt_pose, tf_pt_pose);

				/**** publish roach corners in RVIZ ****/
				geometry_msgs::Point ro_pt;
				ro_pt.x = tf_pt_pose.getOrigin().x(); 
				ro_pt.y = tf_pt_pose.getOrigin().y(); 
				ro_pt.z = tf_pt_pose.getOrigin().z();	
				roach_pts.push_back(ro_pt);
				/***************************************/

				// convert to stupid opencv matrix...
				cv::Mat pt_xyz1 = (cv::Mat_<double>(4,1) << tf_pt_pose.getOrigin().x(), tf_pt_pose.getOrigin().y(), tf_pt_pose.getOrigin().z(), 1.0);
				// make sure to wait until P_ is initialized with message from CameraInfo
				while(P_.rows == 0 && P_.cols == 0){	
					ros::spinOnce();
				}
				// get pixel coordinates of roach corners using projection/camera matrix P_
				cv::Mat trans_pt_xyz1 = P_*pt_xyz1;
				cv::Point2f img_pt(trans_pt_xyz1.at<double>(0,0)/trans_pt_xyz1.at<double>(2,0), trans_pt_xyz1.at<double>(1,0)/trans_pt_xyz1.at<double>(2,0));
				img_corners.push_back(img_pt);
			}
			publishMarkerArray(roach_pts, 0.02, 0.0, 0.0, 1.0);

			cout << "		Pixel-coords Roach corners:" << endl;
			for(int i = 0; i < img_corners.size(); i++){
				cout << "		(" << img_corners[i].x << ", " << img_corners[i].y << ")\n";
			}
			/****************************************************************************************************/
		
			// compute H matrix that goes from real-world coords to pixel coords and H^-1 that goes in opposite direction
			homography_ = computeHomography(roach_corners, img_corners);	
			homography_inverse_ = homography_.inv();

			for(int i = 0; i < homography_.rows; i++){
				for(int j = 0; j < homography_.cols; j++){
					cout << homography_.at<double>(i,j) << " ";
				}
				cout << endl;
			}
		}

		/* Computes homography matrix that transforms from real-life coordinates to pixel coordinates. 
		 */
		cv::Mat computeHomography(vector<cv::Point2f> roach_corners, vector<cv::Point2f> img_corners){
			double x1 = roach_corners[0].x, y1 = roach_corners[0].y,
				   x2 = roach_corners[1].x, y2 = roach_corners[1].y,
				   x3 = roach_corners[2].x, y3 = roach_corners[2].y,
				   x4 = roach_corners[3].x, y4 = roach_corners[3].y;
			double u1 = img_corners[0].x, v1 = img_corners[0].y,
				   u2 = img_corners[1].x, v2 = img_corners[1].y,
				   u3 = img_corners[2].x, v3 = img_corners[2].y,
				   u4 = img_corners[3].x, v4 = img_corners[3].y;
			cv::Mat A = (cv::Mat_<double>(8,8) << x1, y1, 1, 0, 0, 0, -u1*x1, -u1*y1,
												  0, 0, 0, x1, y1, 1, -v1*x1, -v1*y1,
												  x2, y2, 1, 0, 0, 0, -u2*x2, -u2*y2,
												  0, 0, 0, x2, y2, 1, -v2*x2, -v2*y2,
												  x3, y3, 1, 0, 0, 0, -u3*x3, -u3*y3,
												  0, 0, 0, x3, y3, 1, -v3*x3, -v3*y3,
												  x4, y4, 1, 0, 0, 0, -u4*x4, -u4*y4,
												  0, 0, 0, x4, y4, 1, -v4*x4, -v4*y4);
			cv::Mat b = (cv::Mat_<double>(8,1) << u1, v1, u2, v2, u3, v3, u4, v4);
			cv::Mat x = A.inv()*b;
			double h11 = x.at<double>(0,0), h12 = x.at<double>(1,0), h13 = x.at<double>(2,0),
				   h21 = x.at<double>(3,0), h22 = x.at<double>(4,0), h23 = x.at<double>(5,0),
				   h31 = x.at<double>(6,0), h32 = x.at<double>(7,0);
			cv::Mat H = (cv::Mat_<double>(3,3) << h11, h12, h13, 
												  h21, h22, h23, 
												  h31, h32, 1);
			return H;
		}

		/* Converts a 3x3 rotation matrix made up of x_hat, y_hat, z_hat vectors into quaternion.
		 */
		tf::Quaternion rotationToQuaternion(cv::Mat x_hat, cv::Mat y_hat, cv::Mat z_hat){
			double qx, qy, qz, qw;
			double m00 = x_hat.at<double>(0,0), m10 = x_hat.at<double>(1,0), m20 = x_hat.at<double>(2,0),
				   m01 = y_hat.at<double>(0,0), m11 = y_hat.at<double>(1,0), m21 = y_hat.at<double>(2,0),
				   m02 = z_hat.at<double>(0,0), m12 = z_hat.at<double>(1,0), m22 = z_hat.at<double>(2,0);
			double trace = m00 + m11 + m22;
			if(trace > 0){
				double S = 0.5/sqrt(trace+1.0); // S=4*qw
				qw = 0.25/S;
				qx = (m21 - m12)*S;
				qy = (m02 - m20)*S;
				qz = (m10 - m01)*S;			
			}else if((m00 > m11) && (m00 > m22)){
				double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
				qw = (m21 - m12)/S;
				qx = 0.25 * S;
				qy = (m01 + m10)/S; 
				qz = (m02 + m20)/S; 
			}else if(m11 > m22){
				double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
				qw = (m02 - m20)/S;
				qx = (m01 + m10)/S; 
				qy = 0.25 * S;
				qz = (m12 + m21)/S; 
			}else{
				double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
				qw = (m10 - m01)/S;
				qx = (m02 + m20)/S;
				qy = (m12 + m21)/S;
				qz = 0.25 * S;
			}
			tf::Quaternion q(qx, qy, qz, qw);
			return q;
		}

		/* Rotate tf::Vector3 by theta degrees around the z-axis. 
		 */
		tf::Vector3 rotateZ(tf::Vector3 vec, double theta){
			double data[3] = {vec.getX(), vec.getY(), vec.getZ()};
			cv::Mat vecMat = cv::Mat(3,1, CV_64F, data);
			cv::Mat rotMat = (cv::Mat_<double>(3,3) << cos(theta*PI/180), -sin(theta*PI/180), 0.0, sin(theta*PI/180), cos(theta*PI/180), 0.0, 0.0, 0.0, 1.0);
			cv::Mat mult = rotMat*vecMat;
			tf::Vector3 result(mult.at<double>(0,0),mult.at<double>(1,0),mult.at<double>(2,0));
			return result;
		}

		/* Sets up a point cloud representing the camera FOV. 
		 */
		void setCamCloud(){
			for(int u = 0; u < CAMERA_IMG_W; u+=48){
				for(int v = 0; v < CAMERA_IMG_H; v+=48){
					double data[3] = {u, v, 1.0};
					cv::Mat vec = cv::Mat(3,1, CV_64F, data);
					cv::Mat result = homography_inverse_*vec;
					double pt_x = result.at<double>(0,0)/result.at<double>(2,0); 
					double pt_y = result.at<double>(1,0)/result.at<double>(2,0);
					cout << "(u = " << u  << ", v = " << v << "), (pt_x = " << pt_x << ", pt_y = " << pt_y << ")\n";
					hom_cloud_->points[hom_num_pts].x = pt_x;
					hom_cloud_->points[hom_num_pts].y = pt_y;
					hom_cloud_->points[hom_num_pts].z = 0.0;	
					hom_num_pts+=1;
				}
			}
			cout << "Finished setting up all homography plane points..." << endl;
			hom_cloud_->header.stamp = ros::Time::now().toNSec();
		}

		/* Assigns the next goal location by dividing the camera image plane into GOAL_GRID_H*GOAL_GRID_W
		 * grid rectangles, each with 1/GOAL_GRID_H*GOAL_GRID_W probability of being selected.
		 * Uses homography matrix to go from the selected region in the camera image to real-life coordinates
		 * in the exploration plane. 
		 */
		void assignGoal(string ar_marker){
			//cout << "In assignGoal():" << endl;
			double pdf[GOAL_GRID_W*GOAL_GRID_H];
			double prev = 0;
			int i = 0;
			/*** print goal_grid_ ***/
			cout << "GOAL GRID: " << endl;
			for(int row = 0 ; row < GOAL_GRID_H; row++){
				for(int col = 0; col < GOAL_GRID_W; col++){
					pdf[i] = prev+goal_grid_[row][col];
					prev += goal_grid_[row][col];
					i++;
					cout << goal_grid_[row][col] << " ";
				}
				cout << endl;
			}
			srand(time(NULL));
			// get random number between 0.0 and 1.0
			double randNum = randDouble(0.0, 1.0);
			int gridCell = 0; 

			// find which grid cell was chosen
			// CONVENTION: goal_grid enumeration goes from left to right starting at 1 and going to GOAL_GRID_H*GOAL_GRID_W
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
			// convert grid cell to row, col for indexing into image plane
			//gridCell = 10;
			int row = 0, col = 0;
			int tmpCell = gridCell;
			while(tmpCell > GOAL_GRID_W){
				tmpCell -= GOAL_GRID_W;
				row++;
			}
			col = tmpCell-1;
			// store the chosen goal_row and goal_col for future reference
			goal_row_ = row;
			goal_col_ = col;
			// upper left corner of ROI
			double rect_h = (double)CAMERA_IMG_H/(double)GOAL_GRID_H; double rect_w = (double)CAMERA_IMG_W/(double)GOAL_GRID_W;
			double ul_corner_x = col*rect_w; double ul_corner_y = row*rect_h; 
			//cout << "		ul_corner: (" << ul_corner_x << ", " << ul_corner_y << ")\n";

			// store center of ROI 
			cv::Mat centroid = (cv::Mat_<double>(3,1) << ul_corner_x+(rect_w/2.0), ul_corner_y+(rect_h/2.0), 1.0);

			// get real-world (x,y) coordinates using homography inverse
			cv::Mat result = homography_inverse_*centroid;
			double x = result.at<double>(0,0)/result.at<double>(2,0);
			double y = result.at<double>(1,0)/result.at<double>(2,0);

			// set goal point
			goal_pt_.x = x;
			goal_pt_.y = y;
			goal_pt_.z = 0.0;										

			// publish goal marker for rviz visualization
			publishGoalMarker(goal_pt_, 0.05, 0.05, 0.05, 0.0, 1.0, 0.0);	
		}

		/* This function updates the goal grid probabilities given that the robot tried 
		 * to reach a goal_pt_ but was unable to do so after N attempts.
		 * Algorithm:
		 * 		old_prob = get current probability associated with goal_pt_ using goal_row_, goal_col_
		 * 		current probability decreased by (old_prob)^2
		 * 		compute how much need to add to all other grid locations to keep sum(all grid location probabilities) = 1
		 * 		update all other grid locations with need addition
		 */
		void updateGoalGrid(){
			cout << "PREVIOUS GOAL GRID:" << endl;
			for(int row = 0; row < GOAL_GRID_H; row++){
				for(int col = 0; col < GOAL_GRID_W; col++){
					cout << goal_grid_[row][col] << " ";
				}
				cout << endl;
			}
			double old_goal_prob = goal_grid_[goal_row_][goal_col_];
			goal_grid_[goal_row_][goal_col_] -= pow(old_goal_prob,2);
			double addition = (old_goal_prob - goal_grid_[goal_row_][goal_col_])/(GOAL_GRID_W*GOAL_GRID_H-1);
			int count = 0;
			for(int row = 0; row < GOAL_GRID_H; row++){
				for(int col = 0; col < GOAL_GRID_W; col++){
					if(row == goal_row_ && col == goal_col_){
						goal_grid_[row][col] += 0.0;								//TODO stupid hack 
					}else{
						goal_grid_[row][col] += addition;
						count++;
					}
				}
			}
			cout << "changed " << count << " items in the goal_grid table" << endl;
		}

		/* Broadcasts the homography transform and calls the cloud cam publisher 
		 */
		tf::Transform getHomographyTransform(string ar_marker){
			tf::StampedTransform transform;
			try{
			  ros::Time now = ros::Time::now();
			  transform_listener.waitForTransform("usb_cam", ar_marker, now, ros::Duration(5.0));
			  //cout << "		Looking up tf from " << ar_marker << " to usb_cam...\n";
			  transform_listener.lookupTransform("usb_cam", ar_marker, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(10.0).sleep();
			}

			// get lower left-hand corner of the roach 
			tf::Vector3 ll(-0.055, 0.035, -0.08);
			tf::Stamped<tf::Pose> ll_corner(tf::Pose(tf::Quaternion(0, 0, 0, 1), ll), ros::Time(0), ar_marker);

			// publish homography transform
			tf::Transform hom_transform;
			// use lower-left-hand corner of roach in usb_cam frame for translation vector
			hom_transform.setOrigin(tf::Vector3(ll_corner.getOrigin().x(), ll_corner.getOrigin().y(), ll_corner.getOrigin().z()));
			// use quaternion of ar_marker by looking up transform from ar_marker to usb_cam
			hom_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
			return ll_corner;
		}

		/* Runs point cloud publishing and publishes navigation goals for VelociRoACH
		 *
		 * NOTE: roach_control_pub's exploration method needs to run at lower Hz then 
		 * the cloud_goal_publisher for correct synchronization. 
		 */
		void run(std::string pcd_filename){
			string ar_marker = "ar_marker_0_bundle"; 

			tf::Transform hom_transform; 
			tf::StampedTransform usb_hom_transform;

			int input = 'c';
			int maxCloudSize = cloud_->width*cloud_->height;
			bool setGoal = false;
			bool setTransform = false;
			cout << "Press 'q' to quit data collection and save point cloud.\n";

			ros::Rate loop_rate(2);

			/*************** SET UP HOMOGRAPHY & SET INITIAL GOAL FOR VELOCIROACH **************/
			computeHomography(ar_marker);
			hom_transform = getHomographyTransform(ar_marker);
			setCamCloud();

			assignGoal(ar_marker);
			/***********************************************************************************/

			while(nh_.ok() && num_pts < maxCloudSize && input != 'q'){
				input = getch();   // check if user pressed q to quit data collection
				if (input == 'q'){
					break;
				}
				
				if(!setTransform){
					// if we haven't setup the transform successfully yet, attempt to broadcast homography transform with parent ar_marker
					homography_broadcaster_.sendTransform(tf::StampedTransform(hom_transform, ros::Time::now(), ar_marker, "homography_init"));
					try{
					  ros::Time now = ros::Time::now();
					  transform_listener.lookupTransform("usb_cam", "homography_init", ros::Time(0), usb_hom_transform);
					  setTransform = true;
					}
					catch (tf::TransformException ex){
					  ROS_INFO("%s",ex.what());
					}
				}else{
					homography_broadcaster_.sendTransform(tf::StampedTransform(usb_hom_transform, ros::Time::now(), "usb_cam", "homography_plane"));
					// publish point cloud representing FOV of camera
					hom_cloud_pub_.publish(hom_cloud_);

					cout << "In run():" << endl;
					cout << "		success_.data = ";
					if(success_.data)
						cout << "TRUE" << endl;
					else
						cout << "FALSE" << endl;

					/*************** GET GOAL FOR VELOCIROACH **************/
					// if roach reached goal and haven't set new goal
					if(success_.data && !setGoal){ 
						cout << "		CloudGoal: Roach reached goal --> setting new goal..." << endl;
						assignGoal(ar_marker);
						setGoal = true;
					}else if (!success_.data && setGoal){
						setGoal = false;
					}else if(!success_.data && fail_flag_.data && !setGoal){
						cout << "		CloudGoal: Roach FAILED to reached goal --> updating goal grid & setting new goal..." << endl;
						updateGoalGrid();
						assignGoal(ar_marker);
						setGoal = true;
					}
					// publish either updated goal or old goal
					goal_pub_.publish(goal_pt_);

					cout << "		setGoal = ";
					if(setGoal)
						cout << "TRUE" << endl;
					else
						cout << "FALSE" << endl;

					cout << "		fail_flag_.data = ";
					if(fail_flag_.data)
						cout << "TRUE" << endl;
					else
						cout << "FALSE" << endl;
					/*******************************************************/	
				}
				

				/********* PUBLISH POINT CLOUD UNDER VELOCIROACH *********/
				for(double x = -0.055; x <= 0.055; x += 0.02){
					for(double y = -0.035; y <= 0.035; y += 0.02){
						//double x = -0.055; double y = 0.035;
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <termios.h>
#include <time.h>   

#include <coop_pcl/exploration_info.h>

#define PI 3.14159265

// defines how close the roach needs to be to the goal to consider it having reached the goal
// error margin defined in meters
#define EPSILON 0.05			

#define TURN 0
#define FORWARD 1

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeSearch;

using namespace std;

static struct termios oldt, newt;

/* THIS CLASS IS MEANT TO BE USED TOGETHER WITH CLOUD_GOAL_PUB.CPP
 * Together, cloud_goal_pub.cpp publishes goal points for the roach
 * to navigate to while roach_control_pub.cpp publishes the roaches state
 * in navigating to a given goal (i.e. if it suceeded at getting to the
 * goal and needs a new goal or if it is still navigating).
 * roach_control_pub.cpp listens to the goal points delegated by cloud_goal_pub.cpp
 */
class RoachController {

	private: 
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;		// publishes velocity info to /robot1/cmd_vel 
		ros::Publisher success_pub_;		// publishes if succeeded to get to goal /success 
		ros::Subscriber goal_sub_;			// subcribes to /goal from cloud_goal_pub.cpp
		
		geometry_msgs::Point goal_pt_;		// stores goal location for roach to move to
		std_msgs::Bool success_;			// stores if the roach was able to reach goal location

		tf::TransformListener transform_listener;

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

		RoachController(){
			ros::NodeHandle nh;
			nh_ = nh;

			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);
			success_pub_ = nh_.advertise<std_msgs::Bool>("success",1);
			goal_sub_ = nh_.subscribe<geometry_msgs::Point>("goal_pt", 1000, &RoachController::setGoal, this);	

			success_.data = false; 

			goal_pt_.x = -100;
			goal_pt_.y = -100;
			goal_pt_.z = -100;
		}

		/* Callback function used to set the new goal given by cloud_goal_pub
		 */
		void setGoal(const geometry_msgs::Point::ConstPtr& msg){
			//cout << "In setGoal()..." << endl;
			goal_pt_.x = msg->x; 
			goal_pt_.y = msg->y; 
			goal_pt_.z = msg->z; 
		}

		/* Delegates linear and angular velocity commands to navigate roach towards
		 * goal point set by the cloud_goal_pub
		 */
		void explore() {

			// publish Twist message for linear and angular velocities
			geometry_msgs::Twist base_cmd;

			string robot_ar_marker = "ar_marker_0_bundle"; 

			double angle = 0.0;
		    double distance = 0.0;

		    double d2 = 0.0;
		    double b2 = 0.0;

		    int stage = 0;
		    int counter = 0;
		    double Xo = 0.0;
		    double Yo = 0.0;

			double angular = 0.0, linear = 0.0;

			ros::Rate loop_rate(0.5);
			while(nh_.ok()){
				cout << "ROACH_CONTROL_PUB: In explore():\n";
				// get current location of roach
				tf::StampedTransform path_transform;
				try{
				  ros::Time now = ros::Time::now();
				  transform_listener.waitForTransform("usb_cam", robot_ar_marker, now, ros::Duration(5.0));
				  cout << "		Looking up tf from robot_ar_marker to map ...\n";
				  transform_listener.lookupTransform("usb_cam", robot_ar_marker, ros::Time(0), path_transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  ros::Duration(1.0).sleep();
				}
				// get center of roach in context of homography_plane
				tf::Vector3 center_vec(0.055, -0.035, 0.0);
				tf::Stamped<tf::Pose> center(tf::Pose(tf::Quaternion(0, 0, 0, 1), center_vec),ros::Time(0), robot_ar_marker);
				tf::Stamped<tf::Pose> tf_center;
				transform_listener.transformPose("usb_cam", center, tf_center);

				// get goal_points in context of robot_ar_marker
				tf::Vector3 goal_vec(goal_pt_.x, goal_pt_.y, goal_pt_.z);
				tf::Stamped<tf::Pose> goal_pose(tf::Pose(tf::Quaternion(0, 0, 0, 1), goal_vec),ros::Time(0), "usb_cam");
				tf::Stamped<tf::Pose> tf_goal_pose;
				transform_listener.transformPose(robot_ar_marker, goal_pose, tf_goal_pose);

				double goalX = tf_goal_pose.getOrigin().x();
				double goalY = tf_goal_pose.getOrigin().y();
				double goalZ = tf_goal_pose.getOrigin().z();			

				// unit vector in direction of y (the head of the roach)		
				tf::Vector3 unit_y(0.0, 1.0, 0.0);

				// Extract planar position coordinates
    			double Xn= path_transform.getOrigin().x();
   				double Yn= path_transform.getOrigin().y();
				/*
    			cout << "X = " << Xn << ", Y = " << Yn << ", yaw = " << yaw << ", stage = " << stage << ", counter = " << counter << endl;
				*/

				double currX = tf_center.getOrigin().x();											
				double currY = tf_center.getOrigin().y();
				double currZ = tf_center.getOrigin().z();
				cout << "		currPt: (" << currX << ", " << currY << ", " << currZ << ")\n"; 
				cout << "		goalPt: (" << goal_pt_.x << ", " << goal_pt_.y << ", " << goal_pt_.z << ")\n"; 
				if(abs(currX - goal_pt_.x) <= EPSILON && abs(currY - goal_pt_.y) <= EPSILON){
					// roach has reached the goal!
					success_.data = true;
					success_pub_.publish(success_);
					angular = 0.0;
					linear = 0.0;	// don't move until next command
					cout << "		REACHED GOAL!" << endl;
				}else if(goal_pt_.x != -100 && goal_pt_.y != -100 && goal_pt_.z != -100){						//TODO DEAL WITH CASES WHERE CAN NEVER GET TO GOAL (I.E. OBSTACLE IN WAY)
					double goalXLoc = goalX;
					double goalYLoc = goalY;
					// get angle between robot and goal in radians relative to third point (0,0)
					angle = (atan2(unit_y.getY(),unit_y.getX()) - atan2(goalYLoc,goalXLoc)); 
					// get distance between center of robot and goal point
					distance = sqrt(pow(goalXLoc-unit_y.getX(),2)+pow(goalYLoc-unit_y.getY(),2));
					cout << "		angle = " << angle << ", distance = " << distance << endl;
					// Turn stage
					if(stage == TURN) {
						double goal = 0.0;
						double error = angle - goal;
						cout << "goal = " << goal << ", curr angle = " << angle << ", error (i.e. angle-goal) = " << error << endl;
						base_cmd.linear.x = 0.0;
						base_cmd.angular.z = -2.0*error;
						// Threshold the velocity at 0.5 (change the limit if you want to turn quite fast)
						if(base_cmd.angular.z>0.5)
							base_cmd.angular.z=0.5;
						if(base_cmd.angular.z<-0.5)
							base_cmd.angular.z=-0.5;
						cout << "		base_cmd.angular.z = " << base_cmd.angular.z << endl;
						// Check for end condition
						if(error < -0.05 || error > 0.05){
							counter = 0;
							cout << "checked for end condition...\n";
						}else{
							counter++;
							if(counter == 10){
								cout << "		done turning...\n";
								// stop turning
								base_cmd.angular.z = 0.0;
								// movement phase flag
								stage = FORWARD; counter = 0;
								// record start position
								Xo = Xn; Yo = Yn;
							}
						}
					}
					// Forward movement stage
					if(stage == FORWARD){
						double goal = distance;
						double distC = sqrt(pow(Xn-Xo,2) + pow(Yn-Yo,2));
      					double error = goal - sqrt(pow(Xn-Xo,2) + pow(Yn-Yo,2));
						cout << "Distance =" << distC << ", Error = " << error << endl;
						// set speed
						base_cmd.linear.x = 0.1;
						if(error < 0.01 && error > -0.01){
							stage = TURN;
							cout << "GOT TO TARGET!" << endl;
							base_cmd.linear.x = 0.0;
							base_cmd.angular.z = 0.0;
							success_.data = true;
						}
					}
					success_pub_.publish(success_);
				}else{
					// case where we are waiting for goal point to be assigned, do nothing 
					base_cmd.linear.x = 0.0;   
					base_cmd.angular.z = 0.0;  
					success_.data = false;
					success_pub_.publish(success_);
				}
				cout << "		Publishing linear vel: " << linear << ", Angular vel: " << angular << endl;
				// publish velocity commands to get roach towards goal
				cmd_vel_pub_.publish(base_cmd);	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

};

int main(int argc, char** argv){
	// Initialize ROS
	ros::init(argc, argv, "roach_control");

	RoachController controller;
	controller.explore();
}



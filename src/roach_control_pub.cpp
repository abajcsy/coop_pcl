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

#include "tf/LinearMath/Transform.h"

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

			string ar_marker = "ar_marker_0_bundle"; 

			double angle = 0.0;
		    double distance = 0.0;

		    int stage = 0;
		    int counter = 0;
	
			bool got_transform = false;

			ros::Rate loop_rate(0.5);
			while(nh_.ok()){
				cout << "ROACH_CONTROL_PUB: In explore():\n";
				// wait until we successfully looked up transform before doing computations
				if(!got_transform){
					// get current location of roach
					tf::StampedTransform path_transform;
					try{
					  ros::Time now = ros::Time::now();
					  cout << "		Looking up tf from homography_plane to ar_marker...\n";
					  transform_listener.lookupTransform(ar_marker, "homography_plane", ros::Time(0), path_transform);
					  got_transform = true;
					}
					catch (tf::TransformException ex){
					  ROS_INFO("%s",ex.what());														
					}
				}else{
					/* FOR ALL COMPUTATIONS - always operate in ar_marker frame
					 *	(1) Get center of roach wrt ar_marker frame (just 0,0,-0.08)
					 *	(2) Get vector in direction of goal point by transforming goal_pt_ from homography_plane into ar_marker frame
					 *	(3) Compute angle between goal_vec and robot x-axis:
					 *			radians = atan2(tf_goal_pt.getOrigin.y(),tf_goal_pt.getOrigin.x())
					 *  (4) Compute distance between goal_vec and robot x-axis:
					 *  		distance = sqrt(pow(tf_goal_pt.getOrigin.x(),2)+pow(tf_goal_pt.getOrigin.y(),2))
					 */

					// get center of roach in ar_marker frame
					tf::Vector3 roach_center(0.0, 0.0, -0.08);	
					cout << "-----> roach_center (ar_marker frame): (" << roach_center.getX() << ", " << roach_center.getY() << ", " << roach_center.getY() << ")\n";

					// convert goal_pt from homography_plane to ar_marker frame
					tf::Vector3 goal_vec(goal_pt_.x,goal_pt_.y,goal_pt_.z);
					tf::Stamped<tf::Pose> goal_pose(tf::Pose(tf::Quaternion(0, 0, 0, 1), goal_vec),ros::Time(0), "homography_plane");
					tf::Stamped<tf::Pose> tf_goal_pose;
					transform_listener.transformPose(ar_marker, goal_pose, tf_goal_pose);

					// get goal_points in context of homography_plane
					double goalX = tf_goal_pose.getOrigin().x();
					double goalY = tf_goal_pose.getOrigin().y();
					double goalZ = tf_goal_pose.getOrigin().z();	

					cout << "-----> goal_pose (ar_marker frame): (" << goalX << ", " << goalY << ", " << goalZ << ")\n";
	
					if(goal_pt_.x != -100 && goal_pt_.y != -100 && goal_pt_.z != -100){						//TODO DEAL WITH CASES WHERE CAN NEVER GET TO GOAL (I.E. OBSTACLE IN WAY)
						// get angle between robot x-axis and goal in radians 
						angle = atan2(goalY,goalX); 
						// get distance between center of robot and goal point
						distance = sqrt(pow(goalX,2)+pow(goalY,2));
						cout << "		angle = " << angle << ", distance = " << distance << endl;

						// check each time if we're close enough to goal point
						if(abs(distance) < EPSILON){
								stage = TURN;
								cout << "GOT TO GOAL!" << endl;
								base_cmd.linear.x = 0.0;
								base_cmd.angular.z = 0.0;
								success_.data = true;
						}else{
							// Turn stage
							if(stage == TURN) {
								cout << "................IN TURN STAGE................\n";
								double goal = 0.0;
								double error = angle;
								cout << "		goal = " << goal << ", curr angle = " << angle << ", error (i.e. angle-goal) = " << error << endl;
								base_cmd.linear.x = 0.0;
								base_cmd.angular.z = 1*error;
								// Threshold the velocity at 0.5 (change the limit if you want to turn quite fast)
								if(base_cmd.angular.z > 0.4)
									base_cmd.angular.z = 0.4;
								if(base_cmd.angular.z < -0.4)
									base_cmd.angular.z = -0.4;
								if(abs(base_cmd.angular.z) < 0.2){
									if(base_cmd.angular.z < 0.0)
										base_cmd.angular.z = -0.4;
									else
										base_cmd.angular.z = 0.4;
								}
								// Check for end condition
								if(error < -EPSILON || error > EPSILON){
									counter = 0;
									cout << "		checked for end condition...\n";
								}else{
									counter++;
									if(counter == 5){
										cout << "		done turning...\n";
										// stop turning
										base_cmd.angular.z = 0.0;
										// movement phase flag
										stage = FORWARD; 
										counter = 0;
									}
								}
							}
							// Forward movement stage
							if(stage == FORWARD){
								cout << "................IN FORWARD STAGE................\n";
								cout << "Current Distance from Goal =" << distance << endl;
								// set speed
								base_cmd.linear.x = 0.2;
								if(abs(distance) < EPSILON){
									stage = TURN;
									cout << "GOT TO GOAL!" << endl;
									base_cmd.linear.x = 0.0;
									base_cmd.angular.z = 0.0;
									success_.data = true;
								}
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
					cout << "		Publishing linear vel: " << base_cmd.linear.x << ", Angular vel: " << base_cmd.angular.z << endl;
					// publish velocity commands to get roach towards goal
					cmd_vel_pub_.publish(base_cmd);	
				}

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



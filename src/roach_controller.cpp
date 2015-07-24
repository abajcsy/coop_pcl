#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
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

#include <stack>          // std::stack
#include <vector>         // std::vector
#include <deque>          // std::deque

#define PI 3.14159265
#define EPSILON 0.001

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeSearch;

using namespace std;

static struct termios oldt, newt;

class RoachController {
	private: 
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber location_sub_;

		geometry_msgs::Point curr_pt_;
		geometry_msgs::Quaternion curr_orientation_;

		geometry_msgs::Point prev_pt_;

		stack<vector<double> > prev_commands;    // empty stack stores a sequence of the previous linear and angular velocities
		
		double randDouble(double fMin, double fMax) {
			double f = (double)rand() / RAND_MAX;
			return fMin + f * (fMax - fMin);
		}

	public:
		
		void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
			/*cout << "------------------------------------" << endl;
			cout << "New Loc: (" << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << ")\n";
			cout << "Prev Loc: (" << prev_pt_.x << ", " << prev_pt_.y << ", " << prev_pt_.z << ")\n";
			cout << "Curr Loc: (" << curr_pt_.x << ", " << curr_pt_.y << ", " << curr_pt_.z << ")\n";*/

			// deep copy the current point to previous point
			prev_pt_.x = curr_pt_.x; 
			prev_pt_.y = curr_pt_.y; 
			prev_pt_.z = curr_pt_.z; 
			// update new point to be current point
			curr_pt_ = msg->pose.position;
			curr_orientation_ = msg->pose.orientation;

			/*cout << "UPDATED Prev Loc: (" << prev_pt_.x << ", " << prev_pt_.y << ", " << prev_pt_.z << ")\n";
			cout << "UPDATED Curr Loc: (" << curr_pt_.x << ", " << curr_pt_.y << ", " << curr_pt_.z << ")\n";
			cout << "------------------------------------" << endl;*/
		}	

		RoachController(){
			ros::NodeHandle nh;
			nh_ = nh;

			// TODO: change robot1 to robotN where N is the Nth robot that we are controlling
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);
			location_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("location_info", 1000, &RoachController::callback, this);	
		}

		/* Implements a non-blocking getchar() in Linux. Function modifying the terminal settings to 
		 * disable input buffering. 
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

		/* Moves robot with randomly generated linear and angular velocity 
		 */
		void randomWalk() {
			cout << "Robot is starting random walk...\n";
			
			// we will be sending commands of type "Twist"
			geometry_msgs::Twist base_cmd;

			//cout << "Press 'q' to quit moving the robot.\n";
			int input = 'c';

			ros::Rate loop_rate(4);
		    while(nh_.ok() && input != 'q'){
				input = getch();   // check if user pressed q to quit
				if (input == 'q'){
					break;
				}
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

				double randLinX = randDouble(-0.6, 0.6); srand(time(NULL));
				double randAngle = randDouble(-0.6, 0.6); srand(time(NULL));

				//cout << "randLinX: " << randLinX << ", randAngle: " << randAngle << endl;

				base_cmd.linear.x = randLinX;   // move forward
				base_cmd.angular.z = randAngle;   // move left or right
		        cmd_vel_pub_.publish(base_cmd);

				ros::spinOnce();
			}

			// if got command to quit
			if(input == 'q'){
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
				cmd_vel_pub_.publish(base_cmd);
				resetKeyboardSettings();
				cout << "Reset keyboard settings and shutting down.\n";
				ros::shutdown();
			}
		}

		/* Moves robot with randomly generated linear and angular velocity until the
		 * robot gets stuck. If it gets stuck, then it backtracks and continues randomly exploring.
		 */
		void randomSmartWalk(){
			cout << "Robot is starting random smart walk...\n";
			
			geometry_msgs::Twist base_cmd;

			cout << "Press 'q' to quit moving the robot.\n";
			int input = 'c';
			double randLinear = 0; 
			double randAngle = 0;

			bool backtracking = false;
			int num_same_pts = 0; int max_same_pts = 20;
			int backtrack_counter = 0; int max_backtrack = 15;

			ros::Rate loop_rate(0.5);
		    while(nh_.ok() && input != 'q'){
				input = getch();   
				// check if user pressed q to quit
				if (input == 'q'){
					break;
				}
				
				// walk randomly until encounter N of the same consecutive points (i.e. got stuck)
				if(num_same_pts < max_same_pts){
					randLinear = randDouble(0.05, 0.6); srand(time(NULL));
					randAngle = randDouble(-0.6, 0.6); srand(time(NULL));
					// push new command onto stack
					vector<double> cmd;
					cmd.push_back(randLinear);
					cmd.push_back(randAngle);
					// if the two points are the same
					if(abs(curr_pt_.x - prev_pt_.x) < EPSILON && 
						abs(curr_pt_.y - prev_pt_.y) < EPSILON && 
						abs(curr_pt_.z - prev_pt_.z) < EPSILON) { 
						num_same_pts++;
					}else{
						cout << "At new point...adding to command stack..." << endl;
					}
					this->prev_commands.push(cmd);
					cout << "RANDOM WALK --> linear: " << randLinear << ", angle: " << randAngle << endl;	
					backtrack_counter = 0;
				}else if(backtrack_counter < max_backtrack){
					vector<double> prev_cmd = prev_commands.top();
					prev_commands.pop();
					randLinear = -prev_cmd[0];
					randAngle = -prev_cmd[1];
					cout << "BACKTRACKING #" << backtrack_counter << " --> linear: " << randLinear << ", angle: " << randAngle << endl;		
					backtrack_counter++;
				}else{
					num_same_pts = 0;
				}

				base_cmd.linear.x = randLinear;   
				base_cmd.angular.z = randAngle;   
				cmd_vel_pub_.publish(base_cmd);

				ros::spinOnce();
				loop_rate.sleep();
			}

			// if got command to quit
			if(input == 'q'){
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
				cmd_vel_pub_.publish(base_cmd);
				resetKeyboardSettings();
				cout << "Reset keyboard settings and shutting down.\n";
				ros::shutdown();
			}
		}
	

		/* Moves robot towards AR Tag cube goal. 
		 */
		void walkToARGoal(){
			cout << "Robot is starting to walk...\n";
			
			// we will be sending commands of type "Twist"
			geometry_msgs::Twist base_cmd;

			//cout << "Press 'q' to quit moving the robot.\n";
			int input = 'c';

			tf::TransformListener transform_listener;
			string robot_ar_marker = "ar_marker_2"; 
			string target_ar_marker = "ar_marker_0";

			ros::Rate loop_rate(2);
		    while(nh_.ok() && input != 'q'){
				input = getch();   // check if user pressed q to quit
				if (input == 'q'){
					break;
				}
				tf::StampedTransform path_transform;
				try{
				  ros::Time now = ros::Time::now();
				  transform_listener.waitForTransform(robot_ar_marker, target_ar_marker, now, ros::Duration(5.0));
				  cout << "Looking up tf from robot_ar_marker to target_ar_marker ...\n";
				  transform_listener.lookupTransform(robot_ar_marker, target_ar_marker, ros::Time(0), path_transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  ros::Duration(1.0).sleep();
				}
		
				double xLoc = path_transform.getOrigin().x();
				double yLoc = path_transform.getOrigin().y();
				
				double angular = -atan2(yLoc,xLoc)/6;
				double linear = sqrt(pow(xLoc,2)+pow(yLoc,2));

				// threshold velocities if too large
				if(abs(angular) > 1.0){
					cout << "Thresholding angular: " << angular << endl;
					if(angular < 0) 
						angular = -0.7;
					else
						angular = 0.7;
				}
				if(abs(linear) > 1.0){
					cout << "Thresholding linear: " << linear << endl;
					if(linear < 0) 
						linear = -0.7;
					else
						linear = 0.7;
				}

				cout << "linear vel: " << linear << ", angular vel: " << angular << endl;

				base_cmd.linear.x = linear;   // move forward
				base_cmd.angular.z = angular;   // move left or right
		        cmd_vel_pub_.publish(base_cmd);	

				ros::spinOnce();
				loop_rate.sleep();
			}

			// if got command to quit
			if(input == 'q'){
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
				cmd_vel_pub_.publish(base_cmd);
				resetKeyboardSettings();
				cout << "Reset keyboard settings and shutting down.\n";
				ros::shutdown();
			}
		}
};

int main(int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "roach_control");

	RoachController controller;
	controller.randomSmartWalk();
	//controller.walkToARGoal();
	//controller.randomWalk();
}

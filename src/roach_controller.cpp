#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreePC;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeSearch;

using namespace std;

static struct termios oldt, newt;

class RoachController {
	private: 
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		
		double randDouble(double fMin, double fMax) {
			double f = (double)rand() / RAND_MAX;
			return fMin + f * (fMax - fMin);
		}

	public:
			
		RoachController(ros::NodeHandle &nh){
			nh_ = nh;
			// TODO: change robot1 to robotN where N is the Nth robot that we are controlling
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);	
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
		void randomWalk(){
			cout << "Robot is starting random walk...\n";
			
			// we will be sending commands of type "Twist"
			geometry_msgs::Twist base_cmd;

			cout << "Press 'q' to quit moving the robot.\n";
			int input = 'c';
		    while(nh_.ok() && input != 'q'){
				input = getch();   // check if user pressed q to quit
				if (input == 'q'){
					break;
				}
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

				double randLinX = randDouble(0.0, 1.0); srand(time(NULL));
				double randAngle = randDouble(-1.0, 1.0); srand(time(NULL));

				cout << "randLinX: " << randLinX << ", randAngle: " << randAngle << endl;

				base_cmd.linear.x = randLinX;   // move forward
				base_cmd.angular.z = randAngle;   // move left or right
		        cmd_vel_pub_.publish(base_cmd);
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
	ros::NodeHandle nh;

	RoachController controller(nh);
	controller.randomWalk();
}

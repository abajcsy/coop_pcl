Point Cloud Generation and Terrain Navigation Using Cooperative Millirobots
=============

This project includes code and resources for cooperative point cloud generation and terrain exploration with [VelociRoACH] millirobots. In this project, we address the problem of efficiently generating a 3D terrain map by utilizing two inexpensive millirobots, one that acts as a master and the other as an explorer, and a single webcam. Subproblems we address include estimating depth, planning exploration paths, and constructing the 3D terrain map. We take a purely computer vision approach to terrain mapping.  The master robot (represented by the single webcam) watches the explorer robot (the VelociRoACH) walk through the scene. We generate the point cloud based on the explorer robot movement. A human operator can then plan a path for the master robot to safely move through the explored scene. 

The two main cpp files involved are cloud_goal_pub.cpp and roach_control_pub.cpp. Our goal assignment algorithm within cloud_goal_pub.cpp guarantees that the explorer robot is only assigned goal locations within the FOV of the camera. Initially, each location within the FOV is equally likely to be selected as a goal location. However, if the explorer robot fails to successfully reach a goal location within N tries, the probability of assigning that location again decreases by the probability squared. 

The communication between cloud_goal_pub.cpp and roach_control_pub.cpp and the VelociRoACH follows the below diagram:

![alt tag](https://raw.github.com/abajcsy/coop_pcl/master/resources/flowchart.png)

Code is written in C++ and Python and is using [ROS Indigo].

Dependencies
-------
In order to run all the functionality of this project, clone the following projects into your ros workspace: 
* [ar_track_alvar]
* [experiment_calib]
* [pcl]
* [perception_pcl]
* [man_joy_override]

If you're using catkin_make, navigate to the /src directory in your ros workspace and type 'git clone' followed by the http://... website to each of the following github repositories. Then, navigate up one folder level to your ros workspace and type 'catkin_make'.

Example:
```bash
cd ~/ros_workspace/src
git clone https://github.com/abajcsy/coop_pcl
cd ..
catkin_make
```

What You Need
-------
* Single Microsoft Lifecam 3000
* 1+ VelociRoACH controlled with a XBee radio module
* (Preferred AR Tag method) Print out and assemble the AR tag hat for the VelociRoACH under /resources called [roach_hat.png]. NOTE: To use this hat, you must do calibration for the AR tag positions and distances from each other using the [experiment_calib] package. 
* (Alternative AR Tag method) 3.5 x 3.5 x 3.5 cm AR tag cube generated with through [ar_track_alvar] (used for tracking the VelociRoACHes)

Running the ROS Nodes
-------
* To run the project, including AR tracking, cloud_goal_pub, roach_control_pub, and RVIZ nodes:
```bash
roslaunch coop_pcl exploration_test.launch
```

Notes
-------
* (If using AR cube method) to print the AR cube for the back of the VelociRoACH, print the file under resources/MarkerData_0_1_2_3_4_5.png at 30% scale for ar tags at ~3.5 cm (this will ensure the measurements are correct in the ar_cube_bundle.xml file)
* After saving out the point cloud pcd file, use pcl_ros package and view it in rviz with the following command. Note that transforms are not included in the pcd_to_pointcloud visualization, so the point cloud's orientation may not be how it was captured. Set up a static transform between map and usb_cam frame for correct viewing. The "0.1" argument publishes the content of test_pcd.pcd ~10 times a second to the map frame of reference. 
```bash
rosrun pcl_ros pcd_to_pointcloud test_pcd.pcd 0.1 _frame_id:=/map 
```
* To run the octree_visualizer node:
```bash
roslaunch coop_pcl octree_visual_test.launch
```

Arguments
-------
#### cloud_goal_pub.cpp ####
* 'resolution' - Resolution of the octree (in meters). Typically 0.01 is a good place to start for VelociRoACH maps {default: 0.005}
* 'pcd_file' - File path for point cloud .pcd file to be saved to. If user passes in the string "NULL", the point cloud will not be saved. {default: NULL} 
* 'cloud_width' - Number of points in cloud (NOTE: right now all clouds are unordered, so their size is always 1 x cloud_width). {default: 15000}

#### roach_control_pub.cpp ####
* 'robot_name' - Name of robot to publish twist commands to {default: "robot1"}

Extra Info
-------
Developed during Summer 2015 as part of SUPERB-ITS NSF REU Program in the [Biomimetic Millisystems Lab], UC Berkeley. 

[ar_track_alvar]: http://wiki.ros.org/ar_track_alvar
[experiment_calib]: https://github.com/abuchan/experiment_calib 
[pcl]: http://wiki.ros.org/pcl
[perception_pcl]: http://wiki.ros.org/perception_pcl
[man_joy_override]: https://github.com/abuchan/man_joy_override
[roach_hat.png]: https://github.com/abajcsy/coop_pcl/blob/master/resources/roach_hat.png
[VelociRoACH]: http://robotics.eecs.berkeley.edu/~ronf/Ambulation/
[Biomimetic Millisystems Lab]: http://robotics.eecs.berkeley.edu/~ronf/Biomimetics.html
[ROS Indigo]: http://wiki.ros.org/indigo


Author: Andrea Bajcsy
Program: SUPERB-ITS REU, Summer 2015, Biomimetic Millisystems Lab 
Date: 07/13/15

Project: 
This project includes code and resources for cooperative point cloud generation with VelociRoACHes.

Code is written in C++ and Python.

Dependencies:
- ar_track_alvar (http://wiki.ros.org/ar_track_alvar)
- pcl (http://wiki.ros.org/pcl)
- perception_pcl (http://wiki.ros.org/perception_pcl)
- man_joy_override (https://github.com/abuchan/man_joy_override)

What you need:
- Single Microsoft Lifecam 3000
- 1+ VelociRoACH controlled with a Logitech Rumble Gamepad F510 and XBee radio module
- 3.5x3.5x3.5 cm AR tag cube generated with through ar_tracker_alvar (used for tracking the VelociRoACHes)

Notes:
-To print the AR cube for the back of the VelociRoACH, print the file under resources/MarkerData_0_1_2_3_4_5.png at 30% scale for ar tags at ~3.5 cm (this will ensure the measurements are correct in the ar_cube_bundle.xml file)
-After saving out the point cloud pcd file, view it in rviz with the following command: rosrun pcl_ros pcd_to_pointcloud test_pcd.pcd 0.1 _frame_id:=/map. Note that transforms are not included in the pcd_to_pointcloud visualization, so the point cloud's orientation may not be how it was captured. The "0.1" argument publishes the content of test_pcd.pcd ~10 times a second to the map frame of reference. 

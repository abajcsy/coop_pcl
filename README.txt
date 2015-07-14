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
To print the AR cube for the back of the VelociRoACH, print the file under resources/MarkerData_0_1_2_3_4_5.png at 30% scale for ar tags at ~3.5 cm (this will ensure the measurements are correct in the ar_cube_bundle.xml file)

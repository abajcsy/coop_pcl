Cooperative Point Cloud Generation
=============

This project includes code and resources for cooperative point cloud generation and terrain exploration with [VelociRoACHes].

Code is written in C++ and Python and is using [ROS Indigo].


Dependencies
-------
* [ar_track_alvar]
* [pcl]
* [perception_pcl]
* [man_joy_override]

What You Need
-------
* Single Microsoft Lifecam 3000
* 1+ VelociRoACH controlled with a Logitech Rumble Gamepad F510 and XBee radio module
* 3.5x3.5x3.5 cm AR tag cube generated with through ar_track_alvar[link_ar_track_alvar] (used for tracking the VelociRoACHes)

Notes
-------
* To print the AR cube for the back of the VelociRoACH, print the file under resources/MarkerData_0_1_2_3_4_5.png at 30% scale for ar tags at ~3.5 cm (this will ensure the measurements are correct in the ar_cube_bundle.xml file)
* After saving out the point cloud pcd file, use pcl_ros package and view it in rviz with the following command: rosrun pcl_ros pcd_to_pointcloud test_pcd.pcd 0.1 _frame_id:=/map. Note that transforms are not included in the pcd_to_pointcloud visualization, so the point cloud's orientation may not be how it was captured. The "0.1" argument publishes the content of test_pcd.pcd ~10 times a second to the map frame of reference. 
* To run the pcl_publisher:
```bash
roslaunch coop_pcl pcl_test.launch
```
* To run the octree_visualizer:
```bash
roslaunch coop_pcl octree_visual_test.launch
```

Arguments
-------
* 'resolution' - Resolution of the octree (in meters). Typically 0.01 is a good place to start for VelociRoACH maps
* 'pcd_file' - File path for point cloud .pcd file to be saved to or read from. 
* 'cloud_width' - Number of points in cloud (NOTE: right now all clouds are unordered, so their size is always 1 x cloud_width).

Extra Info
-------
Developed during Summer 2015 as part of SUPERB-ITS REU Program in the [Biomimetic Millisystems Lab], UC Berkeley. 

[ar_track_alvar]: http://wiki.ros.org/ar_track_alvar
[pcl]: http://wiki.ros.org/pcl
[perception_pcl]: http://wiki.ros.org/perception_pcl
[man_joy_override]: https://github.com/abuchan/man_joy_override
[VelociRoACHes]: http://robotics.eecs.berkeley.edu/~ronf/Ambulation/
[Biomimetic Millisystems Lab]: http://robotics.eecs.berkeley.edu/~ronf/Biomimetics.html
[ROS Indigo]: http://wiki.ros.org/indigo


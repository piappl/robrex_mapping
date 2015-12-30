RobREx Mapping
==============

Artur Wilkowski  
<ArturWilkowski@piap.pl>

Copyright (C) 2015, Industrial Research Institute for Automation and Measurements  
Security and Defence Systems Division   
<http://www.piap.pl>

Overview
--------

The software is composed of two modules. The first module 'kinect_calib' is used for kinect camera intrinsic and extrinsic parameters calibration. The second module 'surfel_mapper' is dedicated to efficient creation of 3D surfel maps using data from the Kinect sensor. The 'surfel_mapper' module relies on the visual odometry data that can be provided by Ivan Dryanovski 'CCNY RGB-D Tools' (Fast Visual Odometry and Mapping module) [ccny_rgbd_tools](https://github.com/ccny-ros-pkg/ccny_rgbd_tools).

Both 'kinect_calib' and 'surfel_mapper' modules are available in the form of ROS nodes. The 'surfel_mapper' is additionally available as a stand-alone library. 

This code is at an experimental stage, and licensed under the GPLv2 license.

Installing on Ubuntu 14.04
--------------------------

### Prerequisities ###

#### Ubuntu basic modules ####
	sudo apt-get update
	sudo apt-get install ubuntu-desktop,git,terminator

#### ROS Indigo ####

Install ROS Indigo according to [the instructions](http://wiki.ros.org/indigo/Installation/Ubuntu). 

#### ROS workspace ####

Prepare ROS workspace '~/catkin_ws' according to [the instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

#### Additional ROS packages ####

Install required ROS packages

	sudo apt-get install ros-indigo-freenect-launch ros-indigo-libg2o libsuitesparse-dev


#### OpenCV 2.4 with non-free module (for SURF) ####

Update opencv from PPA to the version supporting SURF 

	sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
	sudo apt-get update 
	sudo apt-get install libopencv-nonfree-dev

#### Checkout and build Robrex Mapping module ####
	cd ~/catkin_ws/src
	git clone git@github.com:piappl/robrex_mapping.git
	cd ~/catkin_ws
	catkin_make -DCMAKE_BUILD_TYPE=Release

#### Checkout and build a patched FVOM module ####
	cd ~/catkin_ws/src
	git clone git@github.com:piappl/ccny_rgbd_tools.git
	cd ~/catkin_ws/src/ccny_rgbd_tools
	rosmake

RobREx mapping - Quick usage
----------------------------

#### Provide kinect calibration files ####

Intrinsic and extrinsic Kinect camera calibration files (rgb.yml, depth.yml, trans.yml) should be put in '~/catkin_ws/data/kinect_calib/calibration'. The calibration files can be generated using Kinect Calib module tools. Sample calibration files are available in '~/catkin_ws/src/kinect_calib/samplecalib'.

#### Start Kinect stream preprocessing and registration ####

	roslaunch kinect_calib depthreg.launch

#### Start Fast Visual Odometry and Mapping nodes ####

	roslaunch surfel_mapper visual_odometry.launch 

#### Start Surfel Mapper node ####

	roslaunch surfel_mapper surfel_mapper

surfel_mapper node ROS API 
----------------------

#### Subscribed Topics ####

/mapper_path (nav_msgs/Path)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Sensor positions for subsequent keyframes

/keyframes (sensor_msgs/PointCloud2)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Keyframes to be added to the surfel map

/camera/rgb/camera_info (sensor_msgs/CameraInfo)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Parameters of RGBD camera

#### Published Topics ####

/surfelmap_preview (sensor_msgs/PointCloud2)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Voxel-level preview of the surfel map

/surfelmap (visualization_msgs/MarkerArray)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Part of the surfel map visualized as a marker array

#### Parameters ####

~dmax (double, default:0.05)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;distance threshold for surfel update 

~min_kinect_dist (double, default: 0.8)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;reliable minimum sensor reading distance

~max_kinect_dist (double, default: 4.0)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;reliable maximum sensor reading distance

~octree_resolution (double, default: 0.2)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;resolution of the underlying octree

~preview_resolution (double, default: 0.2)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;resolution of output preview map

~preview_color_samples_in_voxel (int, default: 3)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;number of samples in voxel used for constructing preview point (affects preview efficiency)

~confidence_threshold (int, default: 5)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;confidence threshold used for establishing reliable surfels

~min_scan_znormal (double, default: 0.2)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;acceptable minimum z-component of scan normal

~use_frustum (bool, default: true)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;use frustum or no

~scene_size (int, default: 30000000)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;preallocated size of scene

~logging (bool, default: true)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;logging turned on or off

~use_update (bool, default: true)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;use surfel update or no

#### Services ####

reset_map (surfel_mapper/PublishMap)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Resets map. Removes all surfels, the map is ready to accomodate a new set of keyframes

save_map (surfel_mapper/SaveMap)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Saves the surfel map in the form of XYZRGB point cloud. The default file 'cloud.pcd' is saved to a standard ROS output directory

publish_map (surfel_mapper/PublishMap)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Publishes a fragment of the map as in a \surfelmap topic. The arguments following service call specify x1, x2, y1, y2, z1, z2 coordinates of the map fragment bounding box

Sample calls to services:

Save the current map to a PCD file:

	rosservice call /save_map	

Send the selected map fragment from the bounding box (-0.2, -0.2, 0.6)-(0.2, 0.2, 1.6):

	rosservice call /publish_map -- -0.2 0.2 -0.2 0.2 0.6 1.6

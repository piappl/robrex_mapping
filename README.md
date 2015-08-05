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

#### Checkout and build a patched FVOM module
	cd ~/catkin_ws/src
	git clone git@github.com:piappl/ccny_rgbd_tools.git
	cd ~/catkin_ws/src/ccny_rgbd_tools
	rosmake

Robrex mapping - Quick usage
----------------------------

#### Provide kinect calibration files ####

Intrinsic and extrinsic Kinect camera calibration files (rgb.yml, depth.yml, trans.yml) should be put in '~/catkin_ws/data/kinect_calib/calibration'. The calibration files can be generated using Kinect Calib module tools. Sample calibration files are available in '~/catkin_ws/src/kinect_calib/samplecalib'.

#### Start Kinect stream preprocessing and registration ####

	roslaunch kinect_calib depthreg.launch

#### Start Fast Visual Odometry and Mapping nodes 

	roslaunch surfel_mapper visual_odometry.launch 

#### Start Surfel Mapper node

	roslaunch surfel_mapper surfel_mapper


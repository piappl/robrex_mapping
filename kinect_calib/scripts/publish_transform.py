#!/usr/bin/env python  

#Author: Artur Wilkowsk (PIAP)

PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import argparse
import sys
import yaml
import numpy as np

if __name__ == '__main__':
	rospy.init_node('publish_transform')

	parser = argparse.ArgumentParser(description='Publishes kinect IR to RGB camera transforms')
	parser.add_argument('depth_optical_frame_name', help='name of the depth optical frame')
	parser.add_argument('rgb_optical_frame_name', help='name of the rgb optical frame')
	parser.add_argument('transform_file', help='path to depth_optical_frame -> rgb_optical_frame transformation file')
	args = parser.parse_args(rospy.myargv()[1:]) 

	if len(rospy.myargv()) != 4:
		print 'usage: publish_transform depth_optical_frame_name rgb_optical_frame_name transform_file'
		exit

	#Read data from config file	
	stream = file(args.transform_file, 'r')
	transform_data = yaml.load(stream)

	Rt = np.array(transform_data['rotation']['data']) 
	#R = Rt.reshape(3, 3).transpose()
	R = Rt.reshape(3, 3)
	
	T = np.array(transform_data['translation']['data']) 
	T = T.reshape(3)

	#Transform to quaternion/translation notation
	print 'Publishing rotation matrix R'
	print R
	print 'Corresponding rotation Euler angles'
	print tf.transformations.euler_from_matrix(R)
	print 'Corresponding quaternion'
	print tf.transformations.quaternion_from_euler(*tf.transformations.euler_from_matrix(R))
	print 'Translation vector'
	print T
	
	#Initialize transfrom broadcaster
	br = tf.TransformBroadcaster()
	r = rospy.Rate(2)

	#Broadcast transforms
	while not rospy.is_shutdown():
		br.sendTransform(T,
				     tf.transformations.quaternion_from_euler(*tf.transformations.euler_from_matrix(R)),
				     rospy.Time.now(),
				     args.rgb_optical_frame_name,
				     args.depth_optical_frame_name)
		r.sleep()

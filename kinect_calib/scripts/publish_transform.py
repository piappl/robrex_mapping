#!/usr/bin/env python  

#Author: Artur Wilkowski (PIAP)

PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import sys
import yaml
import numpy as np

if __name__ == '__main__':
	rospy.init_node('publish_transform')

	rgb_optical_frame_name = rospy.get_param('~rgb_optical_frame_name')
	depth_optical_frame_name = rospy.get_param('~depth_optical_frame_name')
	transform_file = rospy.get_param('~transform_file')

	#Read data from config file	
	stream = file(transform_file, 'r')
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
	r = rospy.Rate(30)

	#Broadcast transforms
	while not rospy.is_shutdown():
		br.sendTransform(T,
				     tf.transformations.quaternion_from_euler(*tf.transformations.euler_from_matrix(R)),
				     rospy.Time.now(),
				     rgb_optical_frame_name,
				     depth_optical_frame_name)
		r.sleep()

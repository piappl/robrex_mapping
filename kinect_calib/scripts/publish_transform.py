#!/usr/bin/env python  

#Author: Artur Wilkowski (PIAP)

PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import tf2_ros
import geometry_msgs.msg as gm

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
        stream.close()

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
	br = tf2_ros.TransformBroadcaster()
	r = rospy.Rate(1)

        #Prepare message 
        msg = gm.TransformStamped()
        msg.child_frame_id = rgb_optical_frame_name 
        msg.header.frame_id = depth_optical_frame_name
        msg.transform.translation.x = T[0] 
        msg.transform.translation.y = T[1]
        msg.transform.translation.z = T[2]
        quaternion = tf.transformations.quaternion_from_euler(*tf.transformations.euler_from_matrix(R))
        msg.transform.rotation.x = quaternion[0]
        msg.transform.rotation.y = quaternion[1]
        msg.transform.rotation.z = quaternion[2]
        msg.transform.rotation.w = quaternion[3]

        print 'Message to transmit (except timestamp)'
        print msg

	#Broadcast transforms
	while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now() + r.sleep_dur + r.sleep_dur #Publish one period in the future to prevent 'extrapolation' exceptions (copied from ROS static_transform_publisher)
	    br.sendTransform(msg)
            r.sleep()

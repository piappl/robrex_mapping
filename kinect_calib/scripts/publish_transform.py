#!/usr/bin/env python  
PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import argparse
import sys

if __name__ == '__main__':
	rospy.init_node('publish_transform')

	parser = argparse.ArgumentParser(description='Publishes kinect IR to RGB camera transforms')
	parser.add_argument('depth_optical_frame_name', help='name of the depth optical frame')
	parser.add_argument('rgb_optical_frame_name', help='name of the rgb optical frame')
	parser.add_argument('transform_file', help='path to depth_optical_frame -> rgb_optical_frame transformation file')
	args = parser.parse_args(sys.argv[1:]) 

	if len(rospy.myargv()) != 4:
		print 'usage: publish_transform depth_optical_frame_name rgb_optical_frame_name transform_file'
		exit


	br = tf.TransformBroadcaster()
	r = rospy.Rate(2)

	while not rospy.is_shutdown():
		br.sendTransform((1, 2, 0),
				     tf.transformations.quaternion_from_euler(0, 0, 0.5),
				     rospy.Time.now(),
				     args.depth_optical_frame_name,
				     args.rgb_optical_frame_name)
		r.sleep()

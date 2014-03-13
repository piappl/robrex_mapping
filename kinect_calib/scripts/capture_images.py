#!/usr/bin/env python

PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
from std_msgs.msg import String

# Python libs
import sys, time

# numpy and scipy
import numpy as np
import scipy
from scipy.ndimage import filters
from cv_bridge import CvBridge, CvBridgeError

# OpenCV
import cv2

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge() 

def callbackrgb(data):
	global bridge 
	print 'received image of type: "%s"' % data.header
	try:
		cv_image = bridge.imgmsg_to_cv(data, "bgr8")
	except CvBridgeError, e:
		print e
	cv_image = np.array(cv_image, dtype=np.uint8)
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)

def callbackir(data):
	global bridge 
	print 'received image of type: "%s"' % data.header
	# print 'data type: %s' % data.encoding	
	try:
		cv_image = bridge.imgmsg_to_cv(data, "mono16")
	except CvBridgeError, e:
		print e
	cv_image = np.array(cv_image, dtype=np.uint16)
	#print cv_image
	cv_image8 = cv2.convertScaleAbs(cv_image, alpha=0.5)
	#cv_image8 = cv2.medianBlur(cv_image8,3) 
	cv2.imshow("Image window", cv_image8)
	cv2.waitKey(3)

def listener():

# in ROS, nodes are unique named. If two nodes with the same
# node are launched, the previous one is kicked off. The 
# anonymous=True flag means that rospy will choose a unique
# name for our 'talker' node so that multiple talkers can
# run simultaenously.
	rospy.init_node('listener', anonymous=True)
	#rospy.Subscriber("/camera/rgb/image_color", Image, callbackrgb,  queue_size = 1)
	rospy.Subscriber("/camera/ir/image_raw", Image, callbackir,  queue_size = 1)

    #rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
	cv2.namedWindow("Image window", 1)
	rospy.spin()
        
if __name__ == '__main__':
    listener()

#!/usr/bin/env python

##
# @file publish_images.py
# @author Artur Wilkowski <ArturWilkowski@piap.pl>
# 
# @section LICENSE
#
# Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
# Security and Defence Systems Division <http://www.piap.pl>

PKG = 'kinect_calib' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy

# Python libs
import sys, time

# numpy and scipy
import numpy as np
import scipy
from scipy.ndimage import filters
from cv_bridge import CvBridge, CvBridgeError

# OpenCV
import cv2
import cv

# Ros Messages
from sensor_msgs.msg import Image

import glob 

bridge = CvBridge()

def talker():
	pubir = rospy.Publisher('camera/ir_virtual', Image)
	pubrgb = rospy.Publisher('camera/rgb_virtual', Image)
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(2) # 2hz
	while not rospy.is_shutdown():
		filenames = glob.glob('camerair*')
		filenames.sort() #Some files have shorter usec part so the sorting is not perfect 
		for filenameir in filenames:
			if rospy.is_shutdown(): break #Stop after Ctrl-C
			filenamergb = 'camerargb-' + filenameir[9:]

			#Publish IR image	
			print 'Publishing ' + filenameir 
			cv_image_ir = cv2.imread(filenameir, flags=cv.CV_LOAD_IMAGE_UNCHANGED) ;	
			cv_image_ir = cv.fromarray(cv_image_ir) #Falling back to cv for conversion
			try: 
				messageir = bridge.cv_to_imgmsg(cv_image_ir, "mono8")
			except CvBridgeError, e: print e
			messageir.header.frame_id = '' #for now (works for calibration anyway)
			messageir.header.stamp = rospy.Time.now() 
			pubir.publish(messageir)

			#Publish RGB image
			print 'Publishing ' + filenamergb 
			cv_image_rgb = cv2.imread(filenamergb, flags=cv.CV_LOAD_IMAGE_UNCHANGED) ;	
			cv_image_rgb = cv.fromarray(cv_image_rgb) #Falling back to cv for conversion
			try: 
				messagergb = bridge.cv_to_imgmsg(cv_image_rgb, "bgr8")
			except CvBridgeError, e: print e
			messagergb.header.frame_id = '' #for now
			messagergb.header.stamp = messageir.header.stamp
			pubrgb.publish(messagergb)
			
			r.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass

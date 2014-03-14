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
        
def listener1():
#Initializations
	global bridge 
	rospy.init_node('listener', anonymous=True)
	while not rospy.is_shutdown():
		res = raw_input("Press enter to continue")
#Get IR image
		messageir = rospy.wait_for_message("/camera/ir/image_raw", Image)
		print 'received image of type: "%s"' % messageir.header
		print 'received image width = %s and height = %s' %(messageir.width, messageir.height)
		# print 'data type: %s' % data.encoding	
		try:
			cv_image_ir = bridge.imgmsg_to_cv(messageir, "mono16")
		except CvBridgeError, e:
			print e
		cv_image_ir = np.array(cv_image_ir, dtype=np.uint16)
		cv_image_ir = cv2.convertScaleAbs(cv_image_ir, alpha=1.0)

		#cv_image8 = cv2.medianBlur(cv_image8,3) 

#Get RGB image
		messagergb = rospy.wait_for_message("/camera/rgb/image_color", Image)
		print 'received image of type: "%s"' % messagergb.header
		print 'received image width = %s and height = %s' %(messagergb.width, messagergb.height)
		try:
			cv_image_rgb = bridge.imgmsg_to_cv(messagergb, "bgr8")
		except CvBridgeError, e:
			print e
		cv_image_rgb = np.array(cv_image_rgb, dtype=np.uint8)

#Locate corners
		ir_corners_found, ir_corners = cv2.findChessboardCorners(cv_image_ir, (6,8), flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK);
		cv_image_ir_disp = cv_image_ir.copy() 
		cv2.drawChessboardCorners(cv_image_ir_disp, (6,8), ir_corners, ir_corners_found) 
		print 'IR corners found: %r' % ir_corners_found

		rgb_corners_found, rgb_corners = cv2.findChessboardCorners(cv_image_rgb, (6,8), flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK);
		print 'RGB corners found: %r' % rgb_corners_found
		cv_image_rgb_disp = cv_image_rgb.copy() 
		cv2.drawChessboardCorners(cv_image_rgb_disp, (6,8), rgb_corners, rgb_corners_found) 

#Display images
		cv2.imshow("IR image", cv_image_ir_disp)
		cv2.imshow("RGB image", cv_image_rgb_disp)

		if ir_corners_found and rgb_corners_found:
#Save images
			filenameir = 'camerair-' + str(messageir.header.stamp.secs) + '-' + str(messageir.header.stamp.nsecs) + '.png'	
			print 'Saving ' + filenameir
			cv2.imwrite(filenameir, cv_image_ir)

			filenamergb = 'camerargb-' + str(messageir.header.stamp.secs) + '-' + str(messageir.header.stamp.nsecs) + '.png'	
			print 'Saving ' + filenameir
			cv2.imwrite(filenamergb, cv_image_rgb)
		
		cv2.waitKey(1000)

if __name__ == '__main__':
	try:
		listener1() 
	except rospy.ROSInterruptException: pass

#!/usr/bin/env python
PKG = 'kinect_calib'
import roslib; roslib.load_manifest(PKG)
import yaml
import sensor_msgs.msg
import numpy as np

# Parser function by Stephan (http://answers.ros.org/question/33929/camera-calibration-parser-in-python/)
def parse_yaml(filename):
	stream = file(filename, 'r')
	calib_data = yaml.load(stream)
	cam_info = sensor_msgs.msg.CameraInfo()
	cam_info.width = calib_data['image_width']
	cam_info.height = calib_data['image_height']
	cam_info.K = calib_data['camera_matrix']['data']
	cam_info.D = calib_data['distortion_coefficients']['data']
	cam_info.R = calib_data['rectification_matrix']['data']
	cam_info.P = calib_data['projection_matrix']['data']
	cam_info.distortion_model = calib_data['distortion_model']
	return cam_info

#Main code
if __name__ == "__main__":
	import argparse
	parser = argparse.ArgumentParser(description='Basing on rectification data from stereo calibration restores original camera parameters together with inter-camera transformation')
	parser.add_argument('inputcalibfile1', help='Input calibration file for the first camera in YML format')
	parser.add_argument('inputcalibfile2', help='Input calibration file for the second camera in YML format')
	parser.add_argument('outputcalibfile1', help='Output calibration file for the first camera in YML format')
	parser.add_argument('outputcalibfile2', help='Output calibration file for the second camera in YML format')
	parser.add_argument('outtransformfile', help='Output transformation file for camera1->camera2 transformation')
	args = parser.parse_args()
	try:
		info1 = parse_yaml(args.inputcalibfile1)
		info2 = parse_yaml(args.inputcalibfile2)

		print 'Read the following info from', args.inputcalibfile1, '\n', info1
		print 'Read the following info from', args.inputcalibfile2, '\n', info2

		#Compute intra-camera rotation matrix
		#R = R_2^T * R_1
		R1 = np.array(info1.R) 
		R1 = R1.reshape(3, 3)
		
		R2 = np.array(info2.R) 
		R2 = R2.reshape(3, 3) 

		R = R2.transpose().dot(R1) 

		#Compute intra-camera translation vector
		#T = R_2^T * P_2(4,4) / f (f is the rectified camera focal length)
		P2 = np.array(info2.P)
		P2 = P2.reshape(3, 4)
		T = P2[:,3] / P2[0,0]
		T = R2.transpose().dot(T)

		#Point coordintates in the 1-st camera reference frame are related to point coordinates in the 2-nd camera r.f. by
		#P_1 = R^T * P_2 + T

		#"De-rectify" calibration parameters
	
	except Exception, e:
		import traceback
		traceback.print_exc()

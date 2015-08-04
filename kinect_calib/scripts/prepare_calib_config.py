#!/usr/bin/env python

##
# @file prepare_calib_config.py 
# @author Artur Wilkowski <ArturWilkowski@piap.pl>
# 
# @section LICENSE
#
# Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
# Security and Defence Systems Division <http://www.piap.pl>

PKG = 'kinect_calib'
import roslib; roslib.load_manifest(PKG)
import yaml
import sensor_msgs.msg
import numpy as np

#From http://stackoverflow.com/questions/16782112/can-pyyaml-dump-dict-items-in-non-alphabetical-order
from collections import OrderedDict
class UnsortableList(list):
    def sort(self, *args, **kwargs):
        pass

class UnsortableOrderedDict(OrderedDict):
    def items(self, *args, **kwargs):
        return UnsortableList(OrderedDict.items(self, *args, **kwargs))
#End

# Parser function by Stephan (http://answers.ros.org/question/33929/camera-calibration-parser-in-python/)
# Unused now
def parse_yaml(filename):
	stream = file(filename, 'r')
	calib_data = yaml.load(stream)
	print calib_data
	cam_info = sensor_msgs.msg.CameraInfo()
	cam_info.width = calib_data['image_width']
	cam_info.height = calib_data['image_height']
	cam_info.K = calib_data['camera_matrix']['data']
	cam_info.D = calib_data['distortion_coefficients']['data']
	cam_info.R = calib_data['rectification_matrix']['data']
	cam_info.P = calib_data['projection_matrix']['data']
	cam_info.distortion_model = calib_data['distortion_model']
	return cam_info

def convert_to_ordereddict(indict):
	resdict = UnsortableOrderedDict() 
	resdict['image_width'] = indict['image_width']
	resdict['image_height'] = indict['image_height']

	resdict['camera_name'] = indict['camera_name']

	resdict['camera_matrix'] = UnsortableOrderedDict()
	resdict['camera_matrix']['rows'] = indict['camera_matrix']['rows']
	resdict['camera_matrix']['cols'] = indict['camera_matrix']['cols']
	resdict['camera_matrix']['data'] = indict['camera_matrix']['data']

	resdict['distortion_model'] = indict['distortion_model']

	resdict['distortion_coefficients'] = UnsortableOrderedDict()
	resdict['distortion_coefficients']['rows'] = indict['distortion_coefficients']['rows']
	resdict['distortion_coefficients']['cols'] = indict['distortion_coefficients']['cols']
	resdict['distortion_coefficients']['data'] = indict['distortion_coefficients']['data']

	resdict['rectification_matrix'] = UnsortableOrderedDict() 
	resdict['rectification_matrix']['rows'] = indict['rectification_matrix']['rows']
	resdict['rectification_matrix']['cols'] = indict['rectification_matrix']['cols']
	resdict['rectification_matrix']['data'] = indict['rectification_matrix']['data']
	
	resdict['projection_matrix'] = UnsortableOrderedDict() 
	resdict['projection_matrix']['rows'] = indict['projection_matrix']['rows']
	resdict['projection_matrix']['cols'] = indict['projection_matrix']['cols']
	resdict['projection_matrix']['data'] = indict['projection_matrix']['data']

	return resdict	

def derectify(in_calib_data):
	#Set camera projection matrix to camera matrix (plus zeroes) and rectification matrix to I
	out_calib_data = in_calib_data

	zero3 = np.array(np.mat('0 0 0'))
	K = np.array(out_calib_data['camera_matrix']['data'])
	K = K.reshape(3, 3) 

	P = np.concatenate((K, zero3.transpose()), axis=1) 

	out_calib_data['rectification_matrix']['data'] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
	out_calib_data['projection_matrix']['data'] = P.reshape(12).tolist() 

	return out_calib_data

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
		yaml.add_representer(UnsortableOrderedDict, yaml.representer.SafeRepresenter.represent_dict)

		stream = file(args.inputcalibfile1, 'r')
		calib_data1 = yaml.load(stream)
		stream = file(args.inputcalibfile2, 'r')
		calib_data2 = yaml.load(stream)
		
		#info1 = parse_yaml(args.inputcalibfile1)
		#info2 = parse_yaml(args.inputcalibfile2)

		#print 'Read the following info from', args.inputcalibfile1, '\n', calib_data1 
		#print 'Read the following info from', args.inputcalibfile2, '\n', calib_data2 

		#Compute intra-camera rotation matrix
		#R = R_2^T * R_1
		R1 = np.array(calib_data1['rectification_matrix']['data']) 
		R1 = R1.reshape(3, 3)
		
		R2 = np.array(calib_data2['rectification_matrix']['data']) 
		R2 = R2.reshape(3, 3) 

		R = R2.transpose().dot(R1) 

		#Compute intra-camera translation vector
		#T = R_2^T * P_2(4,4) / f (f is the rectified camera focal length)
		P2 = np.array(calib_data2['projection_matrix']['data'])
		P2 = P2.reshape(3, 4)
		T = P2[:,3] / P2[0,0]
		T = R2.transpose().dot(T)

		#Point coordintates in the 2-nd camera reference frame are related to point coordinates in the 1-st camera r.f. by
		#P_2 = R^T * P_1 + T

		#"De-rectify" calibration parameters
		calib_data1 = derectify(calib_data1) 
		calib_data2 = derectify(calib_data2)

		#Save derectified calibration parameters
		stream = file(args.outputcalibfile1, 'w')
		yaml.dump(convert_to_ordereddict(calib_data1), stream)
		stream.close()
		
		stream = file(args.outputcalibfile2, 'w')
		yaml.dump(convert_to_ordereddict(calib_data2), stream)
		stream.close()

		#Save intra-frame transformation parameters
		stream = file(args.outtransformfile, 'w')
		transformdict = UnsortableOrderedDict() 
	
		transformdict["rotation"] = UnsortableOrderedDict()  
		transformdict["rotation"]["rows"] = 3
		transformdict["rotation"]["cols"] = 3
		transformdict["rotation"]["data"] = R.reshape(9).tolist()
		
		transformdict["translation"] = UnsortableOrderedDict()  
		transformdict["translation"]["rows"] = 1
		transformdict["translation"]["cols"] = 3
		transformdict["translation"]["data"] = T.reshape(3).tolist() 

		#print yaml.dump(transformdict)
		yaml.dump(transformdict, stream)
		stream.close()
	
	except Exception, e:
		import traceback
		traceback.print_exc()

#!/usr/bin/env python

##
# @file sequence_publisher.py 
# @author Maciej Stefanczyk <maciek.slon@gmail.com>
# 
# @section LICENSE
#
# Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
# Security and Defence Systems Division <http://www.piap.pl>

import roslib
roslib.load_manifest('img_tools')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg
import tf

import yaml
import os

class sequence_publisher:

  def __init__(self):
    self.image_ir_pub = rospy.Publisher("image_ir", Image, queue_size=10)
    self.image_co_pub = rospy.Publisher("image_color", Image, queue_size=10)
    self.camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    

    self.bridge = CvBridge()
    
    self.rate = float(rospy.get_param('~rate', '1.0'))
    self.path = rospy.get_param('~path', '.')
    self.calibpath = rospy.get_param('~calib_path', self.path)
    
    self.loop = bool(rospy.get_param('~loop', False))

    self.ir_frame = rospy.get_param('~depth_frame_id', 'depth_frame')
    self.rgb_frame = rospy.get_param('~rgb_frame_id', 'rgb_frame')

    self.publish_pos = rospy.get_param('~publish_pos', True) 
    self.offset = rospy.get_param('~offset', 0) 
    
    print self.rgb_frame
    print self.ir_frame

    self.images = self.load_names(self.path)
    if self.publish_pos:
        self.positions = self.load_positions(self.path)
    self.cur_im = self.offset 
    self.cur_pos = 0

    self.ci = self.parse_yaml(os.path.join(self.calibpath, "rgb.yml"))
    self.ci.header.frame_id = self.rgb_frame

  def parse_yaml(self,filename):
    if filename == '':
      return sensor_msgs.msg.CameraInfo()

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

  def load_names(self, base_path):
    ret = []
    ass_file = open(os.path.join(base_path, "associations.txt"))
 #   print "Loaded names"
    for line in ass_file:
      elem = {}
      names = line.strip("\r\n").split(" ")
      elem['rgb'] = names[3]
      elem['depth'] = names[1]
      ret.append(elem)
#    print ret
    return ret
      
  def load_positions(self, base_path):
    ret = []
    ass_file = open(os.path.join(base_path, "trajectory.txt"))
 #   print "Loaded names"
    for line in ass_file:
      elem = {}
      names = line.strip("\r\n").split(" ")
      elem['tx'] = float(names[1])
      elem['ty'] = float(names[2])
      elem['tz'] = float(names[3])
      elem['qx'] = float(names[4])
      elem['qy'] = float(names[5])
      elem['qz'] = float(names[6])
      elem['qw'] = float(names[7])
      ret.append(elem)
    #print ret
    return ret

  def send_next_image(self):
    if self.cur_im >= len(self.images) and self.loop:
      self.cur_im = self.offset 
      self.cur_pos = 0
    
    if self.cur_im >= len(self.images) or (self.publish_pos and self.cur_pos >= len(self.positions)):
        print "Finished!"
        return False
      
    im_ir = os.path.join(self.path,self.images[self.cur_im]['depth'])
    im_co = os.path.join(self.path,self.images[self.cur_im]['rgb'])
    self.cur_im = self.cur_im + 1

    curtime = rospy.Time.now()
    #Sending image messages
    try:
      img_ir = cv2.imread(im_ir, cv2.IMREAD_UNCHANGED) / 5
      #print img_ir[0][0]
      #img_ir.shape = (img_ir.shape[0],img_ir.shape[1],1)
      img_co = cv2.imread(im_co, cv2.IMREAD_UNCHANGED)
      im_ir_msg = self.bridge.cv2_to_imgmsg(img_ir)
      im_ir_msg.header.stamp = curtime
      im_ir_msg.header.frame_id = self.ir_frame
      self.image_ir_pub.publish(im_ir_msg)
      #self.image_ir_pub.publish(self.bridge.cv2_to_imgmsg(img_ir, encoding="passthrough"))
      img_co_msg = self.bridge.cv2_to_imgmsg(img_co, 'bgr8')
      img_co_msg.header.stamp = curtime;
      img_co_msg.header.frame_id = self.rgb_frame;
      self.image_co_pub.publish(img_co_msg)
      
      self.ci.header.stamp = curtime
      self.camera_info_pub.publish(self.ci)
      
      #print "IR: " + im_ir
      #print "CO: " + im_co
    except CvBridgeError, e:
      print e
      return False

    if self.publish_pos:
      if self.cur_pos >= len(self.positions):
        print "Error: not enough position entries!"
        return False

      #Sending sensor position (transformation)
      pos = self.positions[self.cur_pos]
      br = tf.TransformBroadcaster()
      br.sendTransform((pos['tx'], pos['ty'], pos['tz']),
                       (pos['qx'], pos['qy'], pos['qz'], pos['qw']),
                       curtime,
                       self.ir_frame,
                       "odom")
      self.cur_pos = self.cur_pos + 1
    return True

def main(args):
  rospy.init_node('sequence_publisher', anonymous=True)
  ic = sequence_publisher()
  rospy.sleep(2) #Give some time after advertising
  print 'Length = ' + str(len(ic.images))
  try:
    while not rospy.is_shutdown():
      if not ic.send_next_image():
        break
      print "Sending: " + str(ic.cur_im - 1) 
      rospy.sleep(ic.rate)
    while not rospy.is_shutdown(): #Sleep until shutdown, but sending camera info
      ic.ci.header.stamp = rospy.Time.now() 
      ic.camera_info_pub.publish(ic.ci)
      rospy.sleep(0.1) 
    print "Exit!"
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

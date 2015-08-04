#!/usr/bin/env python

##
# @file trajectory_capture.py 
# @author Artur Wilkowski <ArturWilkowski@piap.pl>
# 
# @section LICENSE
#
# Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
# Security and Defence Systems Division <http://www.piap.pl>

import roslib
roslib.load_manifest('img_tools')
import sys
import rospy
from nav_msgs.msg import Path 

import yaml
import os

class trajectory_capture:
    def pathcallback(self, data):
        self.mapper_path = data 

    def savepath(self, filename):
        f = open(filename, 'w') 
        for pose in self.mapper_path.poses:
            f.write(str(pose.header.seq) + ' ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + ' ' + str(pose.pose.position.z) + ' ' + \
                    str(pose.pose.orientation.x) + ' ' + str(pose.pose.orientation.y) + ' ' + str(pose.pose.orientation.z) + ' ' + str(pose.pose.orientation.w) + '\n')
            #print pose.header.seq
            #print pose.pose.position.x
            #print pose.pose.position.y
            #print pose.pose.position.z
            #print pose.pose.orientation.x
            #print pose.pose.orientation.y
            #print pose.pose.orientation.z
            #print pose.pose.orientation.w
        f.close() 

    def __init__(self):
        self.mapper_path_sub = rospy.Subscriber('mapper_path', Path, self.pathcallback)
        self.mapper_path = None


def main(args):
    rospy.init_node('trajectory_capture', anonymous=True)
    ic = trajectory_capture()
    rospy.spin() 

    print 'Saving ' + 'odompath.txt' + ' on exit.' 
    ic.savepath('odompath.txt') 
    
if __name__ == '__main__':
    main(sys.argv)

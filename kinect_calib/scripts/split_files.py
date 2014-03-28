#!/usr/bin/env python

import sys
import argparse

PKG = 'kinect_calib'

parser = argparse.ArgumentParser(description='Splits stere calibration ost.txt file into two ini files separately for each camera')
parser.add_argument('filein', help='input file')
parser.add_argument('fileout1', help='first output file')
parser.add_argument('fileout2', help='second output file')
args = parser.parse_args() 

if len(sys.argv) != 4:
	print 'usage: kinect_calib filein fileout1 fileout2'
	exit

f = open(args.filein)
lines = f.readlines() 
f.close() 

f1 = open(args.fileout1, 'w')
i = 0
#Looking for the first '# oST'
while lines[i][0:5] != '# oST': 
	i = i + 1

f1.write(lines[i])
i += 1

#Looking for the second '# oST'
while lines[i][0:5] != '# oST': 
	f1.write(lines[i])
	i = i + 1
f1.close()
print "Saved " + args.fileout1

f2 = open(args.fileout2, 'w')
#Looking for the end of file
while i < len(lines):
	f2.write(lines[i])
	i = i + 1
f2.close()
print "Saved " + args.fileout2

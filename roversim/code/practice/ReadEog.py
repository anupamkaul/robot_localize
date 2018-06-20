#!/usr/bin/python

# simple script to read images using eog
# I dont know how to end this script yet.

import os
path = '.'

files = os.listdir(path)

for nextfile in files:
	cmd = "eog " + nextfile
	#print (cmd)
	os.system(cmd)





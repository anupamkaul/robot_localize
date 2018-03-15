import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

filename = 'robocam_example_grid1.jpg'
image = mpimg.imread(filename) # read a 3 color image
plt.imshow(image)
plt.show()

''' turns out that the source grid (1m square) coordinates are:
source coordinates (as measured from example_grid1.jpg)

TL: x=116.274, y=97.9516 [231, 212, 197]
TR: x=195.984, y=97.3065 [191, 165, 155]  (lets make y constant)
BL: x=0.5,     y=141.177 [54, 36, 24]
BR: x=289.177, y=141.823 [45, 31, 22]     (lets make y constant)

destination coordinates (top view square - ideally 1 sq.m pixel will be represented by
1 pixel in the eventual map.

Lets choose a 10 by 10 square pixel coordinate (we dont need roversim for this)
TL: x=116.274, y=97.9516  (start with same TL)
TR: x=126.274, y=97.3065  (add 10 pixels to x tp bring it right)
BL: x=116.274, y=107.9516 (add 10 pixels to y to bring it down, keep same x as BL) 
BR: x=126.274, y=107.3065 (same x as TR, and 10 pixels added to y)
'''

import cv2
import numpy as np

def perspect_transform(img, src, dst):

	# get transform matrix using cv2.getPerspectiveTransform()
	M = cv2.getPerspectiveTransform(src, dst)

	# warp image using cv2.warpPerspective()
	# keep same size as input image
	warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

	return warped

# define source and destination points

source = np.float32([ [116.274,97.9516] , [195.984,97.9516] , [0.5,141.177] , [289.177,141.177] ])
destin = np.float32([ [116.274,97.9516] , [126.274,97.9516] , [116.274,107.9516] , [126.274,107.9516] ])

warped = perspect_transform(image, source, destin)
plt.imshow(warped)
plt.show()

'''
to generate a map now, first apply perspective_transform
and then apply color_threshold to isolate the area. That
is the traversable map from top view ! 
'''

def color_thresh(img, rgb_thresh=(0,0,0)):

	# create an empty array the same size x and y as in image
	# but just a single channel

	color_select = np.zeros_like(img[:,:,0])  # gotta love this function ! 

	# this is how it should ideally be:
	#color_select[:,:,0] = (img[:,:,[0,1,2]] > rgb_thresh) 

	# this could have worked but notice that the dont care on LHS is a different loop than dont care of RHS
	# and there is ambiguity in options here
	# color_select[:,:,0] = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,1] > rgb_thresh[1]) & (img[:,:,2] > rgb_thresh[2]) 

	# its fkin amazing how the variable above_thresh holds a for loop together:

	above_thresh = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,1] > rgb_thresh[1]) & (img[:,:,2] > rgb_thresh[2]) 
	color_select[above_thresh] = 1

	return color_select

# a threshold for ground
r_thold_gnd = 160 
g_thold_gnd = 160
b_thold_gnd = 160
rgb_thold_gnd = (r_thold_gnd, g_thold_gnd, b_thold_gnd) # tuple

colorsel1 = color_thresh(warped, rgb_thold_gnd)
#plt.imshow(colorsel1)
plt.imshow(colorsel1, cmap='gray')
plt.show()

# lets now make sure that the coordinates are with respect to the robot
# this robot centric coordinate system helps describes the env w.r.t robot

# (extract all white coordinates and transform them to 'rover-centric' coordinates)

ypos, xpos = colorsel1.nonzero()
plt.plot(xpos, ypos, '.')
plt.xlim(0, 320)
plt.ylim(0, 160)
plt.show()

# the above gives an 'upside down' image because the origin (0,0) is now
# on the lower left (instead of upper left), and the y-axis reversed.








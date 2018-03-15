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







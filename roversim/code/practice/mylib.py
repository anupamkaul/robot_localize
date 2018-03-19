import cv2
def perspect_transform(img, src, dst):

	# get transform matrix using cv2.getPerspectiveTransform()
	M = cv2.getPerspectiveTransform(src, dst)

	# warp image using cv2.warpPerspective()
	# keep same size as input image
	warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

	return warped


import numpy as np
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

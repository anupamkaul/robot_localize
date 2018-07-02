import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

image = mpimg.imread('robocam_example_grid1.jpg')

plt.imshow(image)
plt.show()

def rover_coords(binary_img):

	# extract xpos, ypos pixel positions from binary_img and
	# convert xpos, ypos to rover_centric coordinates

	x_pixel=0
	y_pixel=0

	# get the non-zero pixels
	xpos, ypos = binary_img.nonzero()

	# calculate the pixel positions with reference to the rover position being at the\
	# center bottom of the image (like it is shown in the simulator usually)

	x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
	y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)

	return x_pixel, y_pixel


# Define calibration box in source (actual) and destination (desired) coordinates
# These source and destination points are defined to warp the image
# to a grid where each 10x10 pixel square represents 1 square meter
dst_size=5

# Set a bottom offset to account for the fact that the bottom of the image 
# is not the position of the rover but a bit in front of it
bottom_offset=6

source = np.float32([ [116.274,97.9516] , [195.984,97.9516] , [0.5,141.177] , [289.177,141.177] ])
destin = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])


# perform warping and color thresholding
import mylib as mlib

warped = mlib.perspect_transform(image, source, destin)
plt.imshow(warped)
plt.show()

colorsel = mlib.color_thresh(warped, rgb_thresh=(160, 160, 160))
plt.imshow(colorsel)
plt.show()


# Extract x and y positions of navigable terrain pixels
# and convert to rover coordinates
xpix, ypix = rover_coords(colorsel)

# Plot the map in rover-centric coords
fig = plt.figure(figsize=(5, 7.5))
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
plt.title('Rover-Centric Map', fontsize=20)
plt.show() 





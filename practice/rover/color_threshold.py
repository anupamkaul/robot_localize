import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

filename = 'robocam_2018_03_08_23_52_27_992.jpg'
image = mpimg.imread(filename) # read a 3 color image
plt.imshow(image)
plt.show()

# the function color_thresh takes the threshold tuple,
# compares it with every pixel's rgb values. If rgb is
# higher than threshold, assign 1 to that pixel, else 0,
# thereby returning a (color tuple thresholded) binary single-layer image (black/white)

# note that for simplicity the output image is a single layer image
# (lets say red layer) where the values for r are 1 or 0

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

# a low threshold
r_thold_low = 25 
g_thold_low = 25
b_thold_low = 25
rgb_thold_low = (r_thold_low, g_thold_low, b_thold_low) # tuple


# an average threshold
r_thold_ave = 100 
g_thold_ave = 100
b_thold_ave = 100
rgb_thold_ave = (r_thold_ave, g_thold_ave, b_thold_ave) # tuple

# a threshold for ground
r_thold_gnd = 160 
g_thold_gnd = 160
b_thold_gnd = 160
rgb_thold_gnd = (r_thold_gnd, g_thold_gnd, b_thold_gnd) # tuple

# pixels below the threshold
colorsel_low = color_thresh(image, rgb_thold_low)
colorsel_ave = color_thresh(image, rgb_thold_ave)
colorsel_gnd = color_thresh(image, rgb_thold_gnd)

plt.imshow(colorsel_low)
plt.show()

plt.imshow(colorsel_ave)
plt.show()

plt.imshow(colorsel_gnd)
plt.show()

# play with plot:
# display the original and modified images side by side
f, (ax1, ax2) = plt.subplots(1, 2, figsize = (21, 7), sharey = True) 
f.tight_layout()
ax1.imshow(image)
ax1.set_title('Original Image', fontsize = 30)

ax2.imshow(colorsel_gnd, cmap='gray') # interesting that cmap = gray actually normalizes that yellow-hue to white (and kills the numbers visual plotter?)
ax2.set_title('Ground Image for Robot', fontsize = 30)

plt.subplots_adjust(left = 0, right = 1, top = 0.9, bottom = 0.)
plt.show()


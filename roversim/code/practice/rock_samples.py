import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

filename = 'robocam_example_grid1.jpg'
image = mpimg.imread(filename) # read a 3 color image
plt.imshow(image)
plt.show()

import mylib as mlib

# define source and destination points

source = np.float32([ [116.274,97.9516] , [195.984,97.9516] , [0.5,141.177] , [289.177,141.177] ])
destin = np.float32([ [116.274,97.9516] , [126.274,97.9516] , [116.274,107.9516] , [126.274,107.9516] ])

# colorsel 'obstacles' ..
#colorsel2 = mlib.color_thresh_range(image, (0, 0, 0), (30, 30, 30))
colorsel2 = mlib.color_thresh_range(image, (0, 0, 0), (165, 42, 42))
plt.imshow(colorsel2, cmap='gray')
#plt.imshow(colorsel2)
plt.show()

warped2 = mlib.perspect_transform(colorsel2, source, destin)
plt.imshow(warped2)
plt.show()
exit()

warped = mlib.perspect_transform(image, source, destin)
plt.imshow(warped)
plt.show()

# a threshold for ground
r_thold_gnd = 160 
g_thold_gnd = 160
b_thold_gnd = 160
rgb_thold_gnd = (r_thold_gnd, g_thold_gnd, b_thold_gnd) # tuple

colorsel1 = mlib.color_thresh(warped, rgb_thold_gnd)
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

# Extract x and y positions of navigable terrain pixels
# and convert to rover coordinates
xpix, ypix = rover_coords(colorsel1)

# Plot the map in rover-centric coords
fig = plt.figure(figsize=(5, 7.5))
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
plt.title('Rover-Centric Map', fontsize=20)
plt.show() 

'''
Notes from the udacity pages:

(rotation -> translation -> clipping if necessary)

for rotation:
x^ = xcos(yaw_angle) - ysin(yaw_angle)
y^ = ycos(yaw_angle) + xsin(yaw_angle)

import numpy as np
#yaw angle is recorded in degrees so first convert to radians

yaw_rad = yaw * np.pi / 180
x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

The next step in mapping to world coordinates is to perform a translation by simply adding the x and y components of the rover's position to the x_rotated and y_rotated values calculated above.

Keep in mind, however, the scale associated with pixels in rover space versus world space. If, for example, as suggested in the previous exercise, you have mapped pixels in rover space such that each represents 0.1 x 0.1 m, and in your world map (as will be the case in the project) each pixel is 1 x 1 m, then you need to divide your rover space pixel values by 10 before mapping to world space. In that case, assuming the x and y position of the rover are given as xpos and ypos:

# Assume a scale factor of 10 between world space pixels and rover space pixels
scale = 10

# Perform translation and convert to integer since pixel values can't be float
x_world = np.int_(xpos + (x_rotated / scale))
y_world = np.int_(ypos + (y_rotated / scale))

Ultimately you would like to add these new pixels to your map, but you may have inadvertently generated values that fall outside your map if the rover was near the edge of the world, so you should also truncate the values to be within the allowable range given by the map size using the np.clip() function.

# Assume a mapsize of 200 x 200 pixels (for our 200 x 200 m world)
world_size = 200
x_pix_world = np.clip(x_world, 0, world_size - 1)
y_pix_world = np.clip(y_world, 0, world_size - 1)

'''

# map to world coordinates now
# we will assume a 'random' yaw angle for now  (see rover_vs_world.png)

rover_yaw = np.random.random(1)*360
print (rover_yaw)

# generate random rover positions in world coordinates (where the rover may be at any given point of time)
# Position values will range from 20 to 180 to avoid 
# the edges in a 200 * 200 pixel world.

rover_xpos = np.random.random(1)*160 + 20
rover_ypos = np.random.random(1)*160 + 20

# note that since we have now chosen random values for yaw and positions,
# multiple runs of the code will result in different outputs each time.

# define the rotation function
def rotate_pix(xpix, ypix, yaw):
	
	# convert yaw angle to radians
	yaw_rad = yaw * np.pi / 180

	xpix_rotated = (xpix*np.cos(yaw_rad) - ypix*np.sin(yaw_rad))
	ypix_rotated = (ypix*np.cos(yaw_rad) + xpix*np.sin(yaw_rad))

	return xpix_rotated, ypix_rotated


#define the translation function
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):

	# apply scaling and translation
	xpix_translated = (xpix_rot / scale) + xpos
	ypix_translated = (ypix_rot / scale) + ypos


	return xpix_translated, ypix_translated


# define the pipeline function to apply rotation and translation (and clipping)
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):

	# apply rotation
	xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)

	# apply translation
	xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)

	# clip to world size
	x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size)
	y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size)

	return x_pix_world, y_pix_world

# Apply this code now ..
# Generate a 200 * 200 pixel worldmap
worldmap = np.zeros((200, 200))
scale = 10

# get navigable pixel positions in world coords
# (where xpix, ypix are rover coordinates from the function rover_coords())

x_world, y_world = pix_to_world(xpix, ypix, rover_xpos, rover_ypos, rover_yaw, worldmap.shape[0], scale)

# Add pixel positions to worldmap
worldmap[y_world, x_world] += 1
print('Xpos rover = ', rover_xpos, 'Ypos rover= ', rover_ypos, 'Yaw = ', rover_yaw)
print('Xworld = ', x_world, 'Yworld= ', y_world) # x_world and y_world are arrays


# Do the showing part now ...

# Plot the map in rover-centric coordinates

f, (ax1, ax2) = plt.subplots(1, 2, figsize = (14, 7))
f.tight_layout()
ax1.plot(xpix, ypix, '.')
ax1.set_title('Rover Space', fontsize=40)
ax1.set_ylim(-160, 160)
ax1.set_xlim(0, 160)
ax1.tick_params(labelsize=20)

ax2.imshow(worldmap, cmap='gray')
ax2.set_title('World Space', fontsize=40)
ax2.set_ylim(0, 200)
ax2.tick_params(labelsize=20)
ax2.set_xlim(0, 200)

plt.subplots_adjust(left=0.1, right=1, top=0.9, bottom=0.1)
plt.show() 








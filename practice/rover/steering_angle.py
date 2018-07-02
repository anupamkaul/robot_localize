# adding some code from concepts 9 and 10 of the "sample and return" project

# we begin autonomous navigation and mapping! Mapping piece was done previously.
# on the map, lets find the best path to traversea

# lets begin by considering an optimal steering angle

# Steering Angle - For this, lets convert our (x,y) (cartesian) to polar (r, theta) coordinates
# (theta will then also be the steering angle, while r will be the distance vector)


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

def to_polar_coords(xpix, ypix):

	dist = np.sqrt(xpix**2 + ypix**2)
	angles = np.arctan2(ypix, xpix)
	
	return dist, angles

# lets use the above function and plot out the direction and angle (steering angle) now:
# --------------------------------------------------------------------------------------


image = mpimg.imread('angle-example.jpg')
plt.imshow(image)
plt.show()

dst_size = 5
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])

import mylib as mlib
warped = mlib.perspect_transform(image, source, destination)
plt.imshow(warped)
plt.show()

colorsel = mlib.color_thresh(warped, rgb_thresh = (160, 160, 160))
plt.imshow(colorsel)
plt.show()

xpix, ypix = mlib.rover_coords(colorsel)
print ("Rover Coordinates: ", xpix, ypix)

distances, angles = to_polar_coords(xpix, ypix)
print ("Polar Coordinates of world map: ", distances, angles)

avg_angle = np.mean(angles)
print ("average steering angle : ", avg_angle)

# Do some plotting
fig = plt.figure(figsize=(12,9))
plt.subplot(221)
plt.imshow(image)
plt.subplot(222)
plt.imshow(warped)
plt.subplot(223)
plt.imshow(colorsel, cmap='gray')
plt.subplot(224)
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
arrow_length = 100
x_arrow = arrow_length * np.cos(avg_angle)
y_arrow = arrow_length * np.sin(avg_angle)
plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=10, width=2)
plt.show()


# final notes from this concept:

'''
The direction (angle) with that represents the average angle of all navigable terrain pixels 
in the rover's field of view is at roughly 0.7 radians in the plot above.

Given the approximate field of view of the camera, the average values for angle that you get 
will vary between about +/- \frac{\pi}/{4} RADIANS 

The input for steering angles you can send to the rover is in DEGREES 
over the range -15 (to the right) to +15 (to the left), so the last step is to choose 
a conversion between your average angle and the steering angle you want to send to the rover. 

The simplest thing to do would be to convert to degrees and clip to the range +/- 15 like this:

avg_angle_degrees = avg_angle * 180/np.pi
steering = np.clip(avg_angle_degrees, -15, 15)

'''









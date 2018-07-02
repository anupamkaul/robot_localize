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

warped = mlib.perspect_transform(image, source, destin)
plt.imshow(warped)
plt.show()

# a threshold for ground
r_thold_gnd = 160 
g_thold_gnd = 160
b_thold_gnd = 160
rgb_thold_gnd = (r_thold_gnd, g_thold_gnd, b_thold_gnd) # tuple

colorsel1 = mlib.color_thresh(warped, rgb_thold_gnd)
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








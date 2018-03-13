# use matplotlib and numpy

import matplotlib.image as mpimg
import matplotlib.pyplot as plt

#%matplotlib inline

#define filename, read and plot the image
#filename = 'robocam_2018_03_08_23_52_43_584.jpg'
filename = 'robocam_2018_03_08_23_52_27_992.jpg'
image = mpimg.imread(filename)
plt.imshow(image)
plt.show()

#the image is now in an array

#explore some image status and re-arrange operations
#using numpy now

import numpy as np
print(image.dtype, image.shape, np.min(image), np.max(image))

# uint8[len of array element] (160[y pixels, width], 320[x pixels, height], 3[channels - rgb]) 0[minvalue] 255[maxvalue]

#extract and plot the 3 channels separately

red_channel   = np.copy(image)
green_channel = np.copy(image)
blue_channel  = np.copy(image)

# in the 3rd dim of image array, index 0=red value, 1=green value, 2=blue value
# we extract a channel by masking all other channel values of pixels of that image to zero

red_channel[:,:,[1,2]] = 0
blue_channel[:,:,[0,1]] = 0
green_channel[:,:,[0,2]] = 0

plt.imshow(red_channel)
plt.show()

plt.imshow(green_channel)
plt.show()

plt.imshow(blue_channel)
plt.show()

# lets instead combine the outputs in subplots
plt.imshow(image)
plt.show()

fig = plt.figure(figsize=(16,4)) # initialize figure area 
plt.subplot(141)                 # plot 1 is 3 col, 1 row
plt.imshow(red_channel)
plt.subplot(142)                 # plot 2 is 3col,  1 row
plt.imshow(green_channel)
plt.subplot(143)                 # plot 3 is 3col, 1 row
plt.imshow(blue_channel)
plt.subplot(144)
plt.imshow(image)

plt.show()

# since the channels have different intensities / hues of color,
# lets isolate color thresholds as an example

# (color_threshold.py)








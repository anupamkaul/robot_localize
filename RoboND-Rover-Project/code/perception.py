import numpy as np
import cv2

# (Anupam)
import matplotlib.pyplot as plt

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels within low and high threshold ranges
def color_thresh_range(img, rgb_thresh_low=(0, 0, 0), rgb_thresh_high=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    within_thresh = (img[:,:,0] > rgb_thresh_low[0]) \
    		& (img[:,:,0] < rgb_thresh_high[0]) \
                & (img[:,:,1] > rgb_thresh_low[1]) \
                & (img[:,:,1] < rgb_thresh_high[1]) \
                & (img[:,:,2] > rgb_thresh_low[2]) \
                & (img[:,:,2] < rgb_thresh_high[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[within_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform
    #source = np.float32([ [116.274,97.9516] , [195.984,97.9516] , [0.5,141.177] , [289.177,141.177] ])
    #destin = np.float32([ [116.274,97.9516] , [126.274,97.9516] , [116.274,107.9516] , [126.274,107.9516] ])

    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destin = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])


    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destin)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    
    nav_terrain  = color_thresh(warped, (160, 160, 160)) # good value for ground pixels

    obstacles    = color_thresh_range(warped, (0, 0, 0), (165, 42, 42)) # brown's RGB  

    rock_samples =  color_thresh_range(warped, (130, 100, 40), (190, 190, 90)) # by visual inspection (EOG)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,0] += obstacles
    Rover.vision_image[:,:,1] += rock_samples
    Rover.vision_image[:,:,2] += nav_terrain 

    # good debug statements..
    #xtemp = plt.imshow(nav_terrain, cmap='gray')
    #plt.show()

    #xtemp = plt.imshow(obstacles)
    #plt.show()

    #xtemp = plt.imshow(rock_samples)  
    #plt.show()

    #Rover.vision_image = warped # this works! on the top half I can continuously see the warped output ...
    #Rover.vision_image = nav_terrain

    # 5) Convert map image pixel values to rover-centric coords
    # (extract x, y coordinates of the navigable terrain and convert it to rover coordinates) ..
    nav_xpix, nav_ypix = rover_coords(nav_terrain)
    obs_xpix, obs_ypix = rover_coords(obstacles)
    rok_xpix, rok_ypix = rover_coords(rock_samples)

    if rock_samples.any():

        #print ("\n\nPERCEP: ROCK SAMPLE SEEN!")

        rover_posx = np.int(Rover.pos[0])
        rover_posy = np.int(Rover.pos[1])
        rock_meanx = np.int(np.mean(rok_xpix))
        rock_meany = np.int(np.mean(rok_ypix))

        '''
        rock_dists = np.sqrt((rover_posx - rock_meanx)**2 + \
                                    (rover_posy - rock_meany)**2)
        

        if (rock_dists < 10):
        '''

        if ((np.absolute(rover_posx - rock_meanx) < 10) or (np.absolute(rover_posy - rock_meany) < 10)):
            print ("\n\nPERCEP: UPCOMING NEAR ROCK SAMPLE SEEN!")

            print("Rover is at ", np.int(Rover.pos[0]), " ", np.int(Rover.pos[1]))
            #print("Mean of rock is at ", np.mean(rok_xpix), " ", np.mean(rok_ypix))
            print("Mean of rock is at ", rock_meanx, " ", rock_meany)
            print("Number of possible pixels: ", len(rok_xpix))
            print (rok_xpix, rok_ypix)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 23 

    # get navigable pixel positions in world coords
    # (where xpix, ypix are rover coordinates from the function rover_coords())

    nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    rok_x_world, rok_y_world = pix_to_world(rok_xpix, rok_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    Rover.worldmap[rok_y_world, rok_x_world, 1] += 1
    Rover.worldmap[nav_y_world, nav_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords (nav_xpix, nav_ypix)


    return Rover

# Define RoverState() class to retain rover state parameters:

import numpy as np

class RoverState():

	def __init__(self):

		self.start_time = None # To record the start time of navigationa
		self.total_time = None # To record total duration of naviagation
		self.img = None # Current camera image
		self.pos = None # Current position (x, y)
		self.yaw = None # Current yaw angle
		self.pitch = None # Current pitch angle
		self.roll = None # Current roll angle
		self.vel = None # Current velocity
		self.steer = 0 # Current steering angle (in degrees)
		self.throttle = 0 # Current throttle value (gas, accel)
		self.brake = 0 # Current brake value
		self.nav_angles = None # Angles of navigable terrain pixels
		self.nav_dists = None # Distances of navigable terrain pixels
		self.ground_truth =  None # ground_truth_3d # Ground truth worldmap
		self.mode = 'forward' # Current mode (can be forward or stop)
		self.throttle_set = 0.2 # Throttle setting when accelerating
		self.brake_set = 10 # Brake setting when braking

        	# The stop_forward and go_forward fields below represent total count
        	# of navigable terrain pixels.  This is a very crude form of knowing
        	# when you can keep going and when you should stop.  Feel free to
        	# get creative in adding new fields or modifying these!
		self.stop_forward = 50 # Threshold to initiate stopping
		self.go_forward = 500 # Threshold to go forward again
		self.max_vel = 2 # Maximum velocity (meters/second)

        	# Image output from perception step
        	# Update this image to display your intermediate analysis steps
        	# on screen in autonomous mode
		self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 

        	# Worldmap
        	# Update this image with the positions of navigable terrain
        	# obstacles and rock samples
		self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 
		self.samples_pos = None # To store the actual sample positions
		self.samples_found = 0 # To count the number of samples found
		self.near_sample = False # Set to True if within reach of a rock sample
		self.pick_up = False # Set to True to trigger rock pickup


Rover = RoverState()

'''
Now you can update Rover with each new set of telemetry values, for example:

Rover.vel = new_velocity_from_telemetry
Rover.yaw = new_yaw_from_telemetry

you can create a decision tree using conditional statements to decide what to do next 
based on current telemetry and / or your analysis, for example:

if Rover.vel >= Rover.max_vel:
    Rover.throttle = 0
else:
    Rover.throttle = Rover.throttle_set
You now have the ability to send commands to the rover to change throttle, brake,
steering angle and take action!

'''



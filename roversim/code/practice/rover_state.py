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

'''
Sending Commands to the Rover
In this project, as in the real world, the perception and decision making steps constitute the bulk of the problem.
In this case, the action step is simple! You'll be provided with a function called send_control() that you can use 
to command the rover and display images to the simulator screen. It looks like this:
'''

# Sending commands to the Rover
# ----------------------------

def send_control(commands, image_string1, image_string2):

	# Define commands to be sent to the rover
	data = {
		'throttle' : commands[0].__str__(),
		'brake': commands[1].__str__(),
		'steering_angle': commands[2].__str__(),
		'inset_image1': image_string1,
		'inset_image2': image_string2, 
	}

	# Send commands via socketIO server
	sio.emit(
		"data",
		data,
		skip_sid = True
	)

'''
This function takes in the argument commands, which is a 3-tuple containing the throttle, brake and steering values 
you have stored in the Rover.throttle, Rover.brake and Rover.steer attributes of your RoverState() object.

The other arguments are called image_string1 and image_string2 and will contain your Rover.vision_image and Rover.worldmap images 
converted to base64 strings that will be rendered onscreen while you're navigating in autonomous mode.

The send_control() function creates a dictionary called data with all of these values then passes this dictionary to the sio.emit() 
function to send commands and images to the rover. You don't need to worry about exactly how this gets accomplished, just that once 
you have updated Rover.throttle, Rover.brake and Rover.steer, your commands input to send_control() will be updated and each time 
you update Rover.vision_image and Rover.worldmap the two display images will be updated. And that's all there is to the action step!

'''


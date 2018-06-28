import numpy as np

RoverPath = {}
prevX = 0
prevY = 0
isOld = 0

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    global prevX
    global prevY
    global isOld

    currX = np.int(Rover.pos[0])
    currY = np.int(Rover.pos[1])
    #pathKey = (np.int(Rover.pos[0]), np.int(Rover.pos[1]))
    pathKey = (currX, currY) # tuple

    if pathKey in RoverPath:
        isOld += 1
        RoverPath[pathKey] += 1

        if (isOld > 150):
            print("Looks like STUCK !! ", Rover.mode, Rover.vel, Rover.throttle, len(Rover.nav_angles))
  
            print("Helping Out ..")
            Rover.steer =- 30 

            #if Rover.vel < Rover.max_vel:
             #   Rover.throttle = Rover.throttle_set
            #else:
             #   Rover.throttle = 0
                
            Rover.throttle = 0  # this works but causes Rover to rotate on spot as throttle is 0. If its not zero, steer doesnt work on a rock.
            Rover.brake = 0

            #Rover.mode = 'stop' #debug this .. goes into mean steer mode (return directly for now)

            isOld = 140 # stall and let the normal code take over next time..
            return Rover 

            # the basic problem is that when stuck in an obstacle, nav_angles is still not zero which causes stop mode to think
            # there is room to move, when in fact there is not.

    else:
        isOld = 0
        print ("ANUPAM: NEW ROV POS: ", np.int(Rover.pos[0]), np.int(Rover.pos[1]))
        RoverPath[pathKey] = 1

        # calc new things on changes only
        if ((currX - prevX) >= 0) & ((currY - prevY) >= 0):
            print("Going TR ..")
        elif ((currX - prevX) <= 0) & ((currY - prevY) >= 0):
            print("Going TL ..")
        elif ((currX - prevX) >= 0) & ((currY - prevY) <= 0):
            print("Going BR ..")
        elif ((currX - prevX) <= 0) & ((currY - prevY) <= 0):
            print("Going BL ..")
        else:
            print("Could not figure out: ", currX, currY, prevX, prevY)

       
    prevX = currX
    prevY = currY

    #print(" !! Path Table: ", RoverPath)

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

		# (Anupam) - Add a directional left bias for 'wall hugging' and see if that improves
                # completing the map scan (quicker)
                Rover.steer += 10

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions

        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            print("In STOP mode")
            if Rover.vel > 0.2:
                print("In STOP mode (1) .. set zero steer")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    print("In STOP mode (2) .. force steer")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    print("In STOP mode (3) .. set mean steer")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                  
                    # (Anupam) - Add directional steer for wall hugging
                    Rover.steer += 10

                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print("Bad Mode..")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover


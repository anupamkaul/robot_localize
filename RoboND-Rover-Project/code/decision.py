import numpy as np

# Determine direction and previously travelled paths
RoverPath = {}
prevX = 0
prevY = 0
isOld = 0

from enum import Enum
class Direction(Enum):
    TopLeft     = 1
    TopRight    = 2
    BottomLeft  = 3
    BottomRight = 4

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
    pathKey = (currX, currY) # tuple

    if (Rover.goal_to_rock):
        Rover.goal_to_rock_steps += 1
        print("Collect Rock Attempt: ", Rover.goal_to_rock_steps)

        if (Rover.goal_to_rock_steps > 50):
            print("Give up Rock Collection Attempt (timeout)")
            Rover.goal_to_rock = False
            Rover.goal_to_rock_steps = 0

    if Rover.rocks_in_path:

        #print ("\n\nDECISION: ROCK SAMPLE SEEN!")

        rover_posx = np.int(Rover.pos[0])
        rover_posy = np.int(Rover.pos[1])
        rock_meanx = abs(np.int(np.mean(Rover.rock_pixels_x)))
        rock_meany = abs(np.int(np.mean(Rover.rock_pixels_y)))

        rock_dist = np.sqrt((rover_posx - rock_meanx)**2 + \
                                    (rover_posy - rock_meany)**2)
        
        if ((np.absolute(rover_posx - rock_meanx) < 30) or (np.absolute(rover_posy - rock_meany) < 30)):

            
            if (len(Rover.rock_pixels_x) > 15):

                print ("\n\nDECISION: UPCOMING NEAR ROCK SAMPLE SEEN!")
                print("Rover is at ", np.int(Rover.pos[0]), " ", np.int(Rover.pos[1]))
                print("Mean of rock is at ", rock_meanx, " ", rock_meany)
                print("Distance from rock ", rock_dist)
                print("Number of possible pixels: ", len(Rover.rock_pixels_x))
                print("Rover velocity ", Rover.vel)
                print("Rover direction ", Rover.direction)
                print (Rover.rock_pixels_x, Rover.rock_pixels_y)

            
            # If samples have been detected by vision, map it to the most known
            # accurate positioning of the rock already present in the world map
              
            rock_world_pos = Rover.worldmap[:,:,1].nonzero()
            if rock_world_pos[0].any():
            
                for idx in range(len(Rover.samples_pos[0])):
                      test_rock_x = Rover.samples_pos[0][idx]
                      test_rock_y = Rover.samples_pos[1][idx]
                      rock_sample_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                                        (test_rock_y - rock_world_pos[0])**2)
                      # Check if rocks were visually detected within 3 meters of known sample positions
                      if np.min(rock_sample_dists) < 3:
                            print("BONGO -- LOCATE_ROCK: NEW ROCK SAMPLE SEEN AT ", test_rock_x, " ", test_rock_y)
                            print("BINGO -- ROVER AT ", rover_posx, " ", rover_posy)
                            rock_meanx = test_rock_x
                            rock_meany = test_rock_y
                            break  # for now quit with the first good result. Needs revisiting.
            
            print("Rover is at ", rover_posx, " ", rover_posy)
            print("Actual Mean of rock is at ", rock_meanx, " ", rock_meany)

            # set steer towards new rock goal and reduce velocity accordingly
            
            # policy : grab rocks first that are along the wall hugging path or straight ahead
            # since we are evaluating in 2D map we need to be cognizant of 'direction' as well..

            if ((Rover.direction is Direction.TopLeft) or (Rover.direction is Direction.TopRight)): 
                if ((rock_meany > rover_posy) and (len(Rover.rock_pixels_x) > 15)):

                    if (Rover.goal_to_rock == False):
                        print("GOAL SET GOING UP - GO FOR ROCK !\n")
                        Rover.goal_to_rock = True

                    # Apply steer, give it time to settle, check again ...

                    print("\n-->ACTION TOP: Set steer towards rock and reduce velocity")
                    if ((rock_meanx - rover_posx) > 0): # rock to the right then steer right
                        print("GOING UP, ROCK TO RIGHT, STEER RIGHT!\n\n")
                        print("old steer: ", Rover.steer, "old vel: ", Rover.vel, "old brake: ", Rover.brake)
                        Rover.steer -= 15
                        Rover.brake += 0.2
                    else:
                        print("GOING UP, ROCK TO LEFT, STEER LEFT!\n\n")
                        print("old steer: ", Rover.steer, "old vel: ", Rover.vel, "old brake: ", Rover.brake)
                        Rover.steer += 15
                        Rover.brake += 0.2

                    #Rover.steer += (rock_meanx - rover_posx) 

                    Rover.vel   -=  np.int((Rover.vel/rock_dist))

                    print("new steer: ", Rover.steer, "new vel: ", Rover.vel, "new brake: ", Rover.brake)

                    '''
                    if ((rock_dist) < 35):
                        Rover.throttle = 0
                        Rover.brake = 1
                        print("brake applied")
                    '''

            if ((Rover.direction is Direction.BottomLeft) or (Rover.direction is Direction.BottomRight)): 
                if ((rock_meany < rover_posy) and (len(Rover.rock_pixels_x) > 15)):

                    if (Rover.goal_to_rock == False):
                        print("GOAL SET GOING DOWN - GO FOR ROCK !\n")
                        Rover.goal_to_rock = True

                    # Apply steer, give it time to settle, check again ...

                    print("\n-->ACTION BOTTOM: Set steer towards rock and reduce velocity")
                    if ((rock_meanx - rover_posx) > 0): # rock to the left then steer left
                        print("GOING DOWN, ROCK TO LEFT, STEER LEFT!\n\n")
                        print("old steer: ", Rover.steer, "old vel: ", Rover.vel, "old brake: ", Rover.brake)
                        Rover.steer += 15
                        Rover.brake += 0.2
                    else:
                        print("GOING DOWN, ROCK TO RIGHT, STEER RIGHT!\n\n")
                        print("old steer: ", Rover.steer, "old vel: ", Rover.vel, "old brake: ", Rover.brake)
                        Rover.steer -= 15
                        Rover.brake += 0.2

                    #Rover.steer += (rock_meanx - rover_posx) 
                    Rover.vel   -=  np.int((Rover.vel/rock_dist))

                    print("new steer: ", Rover.steer, "new vel: ", Rover.vel, "new brake: ", Rover.brake)

                    '''
                    if ((rock_dist) < 35):
                        Rover.throttle = 0
                        Rover.brake = 1
                        print("NEAR ROCK - BRAKE APPLIED")
                    '''


    else:
        #print("SET steer_iter to ZERO")

        # this kills the determined rock collection mode
        if (Rover.goal_to_rock):
            print("Give up Rock Collection Attempt (no rocks in path seen)")
        Rover.goal_to_rock = False
        Rover.goal_to_rock_steps = 0

    # Addionally make use of the Telemetry 'near_sample' data that comes from simulator..
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.goal_to_rock = False

    # Check if near sample
    if Rover.near_sample:
        print(" NEAR SAMPLE ..")
        isOld = 0
        #Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0
        return Rover
    
    # code to determine if rover is stuck, and to 'unstuck' it..    
    if pathKey in RoverPath:
        isOld += 1
        RoverPath[pathKey] += 1

        if (isOld > 100):

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

            isOld = 90 # stall and let the normal code take over next time..
            return Rover 

            # the basic problem is that when stuck in an obstacle, nav_angles is still not zero which causes stop mode to think
            # there is room to move, when in fact there is not.

    else:
        isOld = 0
        RoverPath[pathKey] = 1

        # calc new things on changes only
        if ((currX - prevX) >= 0) & ((currY - prevY) >= 0):
            Rover.direction = Direction.TopRight
        elif ((currX - prevX) <= 0) & ((currY - prevY) >= 0):
            Rover.direction = Direction.TopLeft
        elif ((currX - prevX) >= 0) & ((currY - prevY) <= 0):
            Rover.direction = Direction.BottomRight
        elif ((currX - prevX) <= 0) & ((currY - prevY) <= 0):
            Rover.direction = Direction.BottomLeft
        else:
            print("Could not figure out direction: ", currX, currY, prevX, prevY)
        

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
                if (not Rover.goal_to_rock):
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

		# (Anupam) - Add a directional left bias for 'wall hugging' and see if that improves
                # completing the map scan (quicker)
                if (not Rover.goal_to_rock):
                    Rover.steer += 12

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif ((len(Rover.nav_angles) < Rover.stop_forward) and (not Rover.goal_to_rock)):
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    print("Put Rover in STOP mode! ", len(Rover.nav_angles)) 

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
                    print("Put Rover in Forward mode! ", len(Rover.nav_angles)) 
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                  
                    # (Anupam) - Add directional steer for wall hugging
                    Rover.steer += 12

                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print("Bad Mode..")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If mission is complete then stop for now and await further instructions
    if ((Rover.samples_located == 6) and (Rover.perc_pathmapped > 85)):
        print ("(Decision): Mission Completed. Rocks Located : ", Rover.samples_located, " Area mapped : ", Rover.perc_pathmapped)
    
        Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0
        Rover.mission_complete = 1
    
    return Rover


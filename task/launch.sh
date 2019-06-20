#!/bin/sh

# replace xterm with gnome-terminal or konsole
# a -hold option can be used to keep xterm persistent upon an exception (before -e) 

# House the robot into the world
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" & 
sleep 10 

# -- map creation is one time only, so commenting the stages
# note that map generation does not need the simulation in gazebo to be running

# Create a map of the environment: (run map generation server)
#xterm -hold -e "source devel/setup.bash; gzserver src/pgm_map_creator/world/anupam.world" &
#sleep 5

# Create a map of the environment: (run map client)
#xterm -e "source devel/setup.bash; roslaunch pgm_map_creator request_publisher.launch" &
#sleep 5

xterm -e "source devel/setup.bash; roslaunch my_robot amcl.launch" & 
sleep 5

# Launch the optional teleop keyboard package to move the robot via keyboard 
# instead of via nav move_base in rviz..

xterm -e "source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
sleep 5









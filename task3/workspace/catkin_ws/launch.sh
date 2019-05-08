#!/bin/sh

# replace xterm with gnome-terminal or konsole
# a -hold option can be used to keep xterm persistent upon an exception (before -e) 

xterm -e " source devel/setup.bash; roslaunch my_robot world.launch" & 
sleep 5





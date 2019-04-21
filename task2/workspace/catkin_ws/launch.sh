#!/bin/sh

# replace xterm with gnome-terminal or konsole

xterm -e " source devel/setup.bash; roslaunch my_robot world.launch" & 
sleep 10
xterm -e " source devel/setup.bash; roslaunch ball_chaser ball_chaser.launch" & 
sleep 5
xterm -e "ls" 





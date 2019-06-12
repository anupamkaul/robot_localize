#!/bin/sh

# replace xterm with gnome-terminal or konsole
# a -hold option can be used to keep xterm persistent upon an exception (before -e) 

# House the robot into the world (default world in turtlesim_gazebo is anupam.world)

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & 

#xterm -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world" &

sleep 10 

# make sure teleop is not running when move_base is on
#xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
#sleep 5

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/anupam/robond/task5/workspace/catkin_ws/src/my_robot/maps/map.yaml" & 

#xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl.launch" & 

sleep 5

#xterm -e "cd $(pwd)/../..; source devel/setup.bash ; rosrun rviz rviz -d src/rvizConfig/myslamconfig.rviz" &
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"

sleep 5










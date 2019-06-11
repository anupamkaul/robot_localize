#!/bin/sh

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/anupam/robond/samples/service_robot/workspace/catkin_ws/src/World/UWorld.world" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/anupam/robond/samples/service_robot/workspace/catkin_ws/src/World/Umap.yaml" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" 

#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/UWorld.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/Umap.yaml" &
sleep 5
xterm -e " source /home/workspace/catkin_sw/devel/setup.bash; rosrun pick_objects pick_objects_node" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" 

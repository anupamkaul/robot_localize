#!/bin/sh

# setup up search directories
catkin_dir=/home/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

#Launch turtlebot in my custom world
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/UWorld.world" &
sleep 5

#Launch gmapping demo
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

#Launch Rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#Launch autonomus robot
xterm -e "rosrun wall_follower wall_follower"


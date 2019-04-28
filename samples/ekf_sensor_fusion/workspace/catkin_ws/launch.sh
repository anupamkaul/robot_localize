#!/bin/sh

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10

xterm -e "source devel/setup.bash; roslaunch robot_pose_ekf robot_pose_ekf.launch" &
sleep 5

xterm -e "source devel/setup.bash; rosrun rqt_graph rqt_graph" &




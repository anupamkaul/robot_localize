#!/bin/sh

# setup up search directories
catkin_dir=/home/anupam/robond/samples/service_robot/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

#Launch turtlebot in my custom world
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & #default is anupam.world
sleep 5

#Launch gmapping demo
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

#Launch Rviz
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#Launch autonomus robot
xterm -e "cd $(pwd)/../..; source devel/setup.bash; rosrun wall_follower wall_follower"


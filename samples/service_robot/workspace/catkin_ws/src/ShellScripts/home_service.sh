#!/bin/sh

# setup up search directories
catkin_dir=/home/anupam/robond/samples/service_robot/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/World/UWorld.world" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/World/Umap.yaml" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e "cd $(pwd)/../..; source devel/setup.bash; rosrun home_service home_service_node" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; rosrun pick_objects pick_objects_node" 



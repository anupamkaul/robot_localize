#!/bin/sh

# setup up search directories (replace with /home/workspace/catkin_ws on standard build)
catkin_dir=/home/anupam/robond/task5/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & #default is anupam.world
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/my_robot/maps/map.yaml" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; rosrun pick_objects pick_objects_node" &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" 

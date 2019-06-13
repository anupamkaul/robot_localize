#!/bin/sh

# setup up search directories (replace with /home/workspace/catkin_ws on standard build)
catkin_dir=/home/anupam/robond/task5/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 10 

xterm -e "cd $(pwd)/../..; source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

xterm -e "cd $(pwd)/../..; source devel/setup.bash ; rosrun gmapping slam_gmapping  " &
sleep 5

xterm -e "cd $(pwd)/../..; source devel/setup.bash ; rosrun rviz rviz -d src/rvizConfig/myslamconfig.rviz" &



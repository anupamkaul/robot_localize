#!/bin/sh

# replace xterm with gnome-terminal or konsole
# a -hold option can be used to keep xterm persistent upon an exception (before -e) 

# House the robot into the world
#xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & 
xterm -e "source devel/setup.bash ; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world" &
sleep 10 

# Launch the optional teleop keyboard package to move the robot via keyboard 
# instead of via nav move_base in rviz..

xterm -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

xterm -e "source devel/setup.bash ; rosrun gmapping slam_gmapping  " &
sleep 5

xterm -e "source devel/setup.bash ; rosrun rviz rviz -d src/rvizConfig/myslamconfig.rviz" &
sleep 5

#xterm -e "source devel/setup.bash; roslaunch my_robot mapping.launch" &
#sleep 5

# rtabmap-databaseViewer ~/.ros/rtabmap.db









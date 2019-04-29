#!/bin/sh

# my preferred way of launching because every process gets its own terminal
# hence easier to debug if required. (put a -hold before -e to persist error logs) 

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10

xterm -e "source devel/setup.bash; roslaunch robot_pose_ekf robot_pose_ekf.launch" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch odom_to_trajectory create_trajectory.launch" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

xterm -e "source devel/setup.bash; rosrun rqt_graph rqt_graph" &
sleep 5

xterm -e "source devel/setup.bash; rosrun rviz rviz -d src/EKFLab.rviz" &
#xterm -e "source devel/setup.bash; roslaunch src/RvizLaunch.launch" &
sleep 5




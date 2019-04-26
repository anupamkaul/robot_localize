#!/bin/sh

xterm -e 'source devel/setup.bash ; roslaunch simple_arm robot_spawn.launch' & 
sleep 10 

xterm -e 'source devel/setup.bash ; rqt_image_view /rgb_camera/image_raw' & 
sleep 10

xterm -hold -e 'source devel/setup.bash ; rosservice call /arm_mover/safe_move "joint_1: 0
joint_2: 0"' & 







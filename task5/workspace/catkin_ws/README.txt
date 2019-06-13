Folder "task5" contains my project for the home service robot.

The robot is turtlebot and the world is my original world that I have been using since the first project.

Scripts:
--------
To run this, the main script is home_service.sh (You may need to replace the variable $catkin_dir with a suitable path - probably /home/workspace/catkin_ws)

Other scripts are present in the src/scripts folder:  (to run them please make sure $catkin_dir is set correctly)

1. test_slam.sh - This launches gmapping slam and you can use teleop commands to manually move the robot.

2. wall_follower.sh - This automates map creation via slam by implementing a wall crawling motion for the robot. You can see how the map of the home eventually gets created.  (I have dumped a few screen shots in the Images folder). Please note that while the map is created from slam, the one I finally use is a static pgm map that I created in the localization project previously. The drawback here is that this map has solid walls and no doors (and I did not have time to cut doors manually) which means that I constrain my robot to path planning within a room of choice. (I have tested against various maps though, and they work fine)

3. test_navigation.sh - This launches localization via amcl. I am using the amcl_demo launch file from turtlebot gazebo. Not a whole lot of paramters are modified, because it mostly works as is. I do pass the initial pose to rviz from here. You can also manually invoke move_base from here and see that it works.

4. pick_objects.sh -I created a ros node here that 'automates' move_base by sending out goals. Those goal areas were previously established by add_marker (as I had to constrain my discoveries to within a room of the home). Pick objects has very simple code. It calls action client, and also publishes a topic that informs subscribers that the robot is either in the pickup or dropzone.

5. add_marker.sh - I created a node to interact with markers in rviz. You can run this to visually verify where the goal markers are on rviz. Again very simple code, just cycles through to display, sleep for 5 seconds and then hide the marker.

6. home_service.sh - This culminates and integrates everything (and is also the top level script). It launches another node called home_service that encapsulates the add marker functionality in a class. There are 2 methods employed for discovery of drop and pickup zones. The first method subscribes to the robot's odometry data, and asynchronous to pick_object, whenever this node ascertains that the robot is within the bounds of a 'zone' (with some extra padding for uncertainty) it takes appropriate action. However, many times the odometry data is off. I will debug this further, but I introduced the 2nd method for simplicity - the home_service node subscribes to a topic about zones that pick_object publishes, and uses this synchronicity to take zone based actions.

The images folder contains some snapshots of how the work flowed during the course of the project.

Everything else should be self-explanatory.

-----


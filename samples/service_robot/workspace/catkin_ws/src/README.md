Home Service Robot
=========

Udacity Home Service Robot Project.  This project required a series of modules to allow the robot to navigate a custom environment, find the initial marker, pick it up and move it to selected location.

  #+html:<p align="center"><img src="https://github.com/LeroyFriesenhahn/Home_Service_Robot/blob/master/Final_Project_View.JPG" /><p>

The project consist of the following steps:
  
    1.  Create an environment with the Building Editor in Gazebo.
    2.  Save the environment as pgm and yaml files
    3.  Use the previously create environment to manually test SLAM via teleop
    4.  Create a wall_following node to autonomously map the previously created environment
    5.  Using ROS navigation, manually command the robot to find the 2D Nav Goal in rviz.  (two positions)
    6.  Develop a pick_object node to drive the robot to a pickup and dropoff location
    7   Develop a add_markers node to keep track of the robot by subscribing to the robot odometry.
    8.  Complete the project by connecting all the previously developed nodes.  The allosw the robot to travel to the pickup zone,
        pickup the marker, travel to the dropoff zone and drop the marker.


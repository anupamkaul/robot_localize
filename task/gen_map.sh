# note that map generation does not need the simulation in gazebo to be running

# Create a map of the environment: (run map generation server)
xterm -e "source devel/setup.bash; gzserver src/pgm_map_creator/world/anupam.world" &
sleep 5

# Create a map of the environment: (run map client)
xterm -e "source devel/setup.bash; roslaunch pgm_map_creator request_publisher.launch" &
sleep 5


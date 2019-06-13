#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// create a structure to hold and set the robot goal values
move_base_msgs::MoveBaseGoal setRobotGoal(double x, double y, double z, double w) {

  move_base_msgs::MoveBaseGoal goal;

	// set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  goal.target_pose.pose.orientation.w = w;
  goal.target_pose.pose.orientation.z = z;

  return goal;

}

bool checkRobotGoalStatus(MoveBaseClient& ac, std::string success_msg, std::string fail_msg){
	bool reached_goal = false;

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("%s", success_msg.c_str());
		reached_goal = true;
	}
  else
    ROS_INFO("%s", fail_msg.c_str());

	return reached_goal;
}


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Set the values of the pickup goal and the drop goal
  move_base_msgs::MoveBaseGoal pickup_goal = setRobotGoal(3.4000, -1.5400, 0.4068, 0.73);
  move_base_msgs::MoveBaseGoal drop_goal = setRobotGoal(2.4000, -4.2400, 0.4068, 0.719);

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  bool checkPickupGoal = checkRobotGoalStatus(ac, "In Pickup Zone", "Unable to get to Pickup Zone");
 
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(drop_goal);

  // Wait an infinite time for the results
  ac.waitForResult();
	// Check if the robot reached its goal
	bool checkDropoffGoal = checkRobotGoalStatus(ac, "In Dropoff Zone", "Unable to get to Dropoff Zone");

	// Infom user if both goals were accomplished or not
	if (checkPickupGoal && checkDropoffGoal)
		ROS_INFO("Robot reached both zones successfully!");
	else 
		ROS_INFO("Robot did not reach both zones");

  
  // Sleep for 5 seconds
	ros::Duration(5.0).sleep();


  return 0;
}

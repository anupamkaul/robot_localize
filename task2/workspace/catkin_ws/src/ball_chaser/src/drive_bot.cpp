/* The drive_bot.cpp server node that will provide the ball_chaser/command_robot service. */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

// This callback function executes whenever a command_robot service is requested
bool handle_command_robot_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("command_robot (drive to target) req received - j1:%1.2f, j2:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set wheel velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message (not really checking anything)
    res.msg_feedback = "Robot velocity set - j1: " + std::to_string(motor_command.linear.x) + " , j2: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize this ROS node and define a handler to it
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("drive_bot/command_robot", handle_command_robot_request);
    ROS_INFO("Service drive_bot .. command the robot to move..");

    //Handle ROS communication events
    ros::spin();

    return 0;
}

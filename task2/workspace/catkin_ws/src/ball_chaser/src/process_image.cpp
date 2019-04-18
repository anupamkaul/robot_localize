#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
bool moving_state = false;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Chasing Ball .. Go!");
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x  = lin_x;
    srv.request.angular_z = ang_z; 

    // call the drive_bot/command_robot service and move the bot..
    if(!client.call(srv)) 
        ROS_ERROR("Failed to call drive_bot service to command robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    ROS_INFO("Analyzing Camera Image ..");

    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {

            // on detecting the ball, yap like a puppy and go round-n-round
            ROS_INFO("White ball detected ..");
            drive_robot(0.3, 0.2);
            moving_state = true;
            break;
        }
    }

    // If bot is moving then stop it if it sees no ball..
    if (moving_state) 
    {
        drive_robot(0, 0);
        moving_state = false;
    } 
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/drive_bot/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

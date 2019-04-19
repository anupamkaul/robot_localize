#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
bool moving_state = false;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
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

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // img is repd as a single array rep. with height = no. of rows and step = length of row in bytes
    // I see ht: 800 and step: 2400

    unsigned long int step_left_range  = img.step * 0.34; // 0 to 1/3rd
    unsigned long int step_mid_range   = img.step * 0.67; // 1/3rd to 2/3rd
    unsigned long int step_right_range = img.step;        // 2/3rd to end


    for (int i = 0; i < img.height * img.step; i++) {

        if (img.data[i] == white_pixel) {

            // on detecting the ball, yap like a puppy and go round-n-round
            ROS_INFO_STREAM("Analyzing Camera Image (image params)..\n ht (no. of rows) " << img.height << " width (no. of cols) " << img.width << "\n and step (full length of row in bytes) " << img.step << "\n \n ");

            ROS_INFO_STREAM("\n White ball detected .. pixel no. " << i);

            int current_row = round((i+1)/img.step);

            int current_left_low  = img.step * current_row; 
            int current_left_high = step_left_range + (img.step * current_row); 

            int current_mid_low   = (current_left_high + 1); 
            int current_mid_high  = (step_mid_range)+ (img.step * current_row); 

            int current_right_low = (current_mid_high + 1); 
            int current_right_high = (step_right_range) + (img.step * current_row);

            ROS_INFO_STREAM("We are in Row: " << current_row);   
            ROS_INFO_STREAM("\n left-low: " << current_left_low << " left-high" << current_left_high);   
            ROS_INFO_STREAM("\n mid-low: " << current_mid_low << " mid-high" << current_mid_high);   
            ROS_INFO_STREAM("\n right-low: " << current_right_low << " right-high" << current_right_high);   

            if ((i >= current_left_low) && (i <= current_left_high)) {

                if (i < 3) { // probably touching ball, stop moving!
 
                    ROS_INFO("Close to Ball, Stop!");
                    drive_robot(0, 0);
                    moving_state = false;
                } 
                else {

                    ROS_INFO("Ball at LEFT, Steer LEFT");
                    drive_robot(0.3, 0.2);
                    moving_state = true;

                }

            } 
            else if ((i >= current_mid_low) && (i <= current_mid_high)) {

                    ROS_INFO("Ball in CENTER, Move Forward");
                    drive_robot(0.3, 0);
                    moving_state = true;

            }
            else if ((i >= current_right_low) && (i <= current_right_high)) {

                    ROS_INFO("Ball at RIGHT, Steer RIGHT");
                    drive_robot(0.3, -0.2);
                    moving_state = true;

            }
            else 
                ROS_INFO("Unknown Direction !!\n"); // take no action (inertial) 

            return; // exit and recalculate on next camera image
        }
    }

    // If bot is moving then stop it if it sees no ball..
    if (moving_state) 
    {
        ROS_INFO("\n No White ball .. Don't move");
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
    ROS_INFO("Process Image Node .. ");

    // Handle ROS communication events
    ros::spin();

    return 0;
}

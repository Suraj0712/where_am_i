#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    // service call
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int left_count = 0;
    int mid_count = 0;
    int right_count = 0;

    for(int h=0; h<img.height; h++)
    {     
        // Data  from sensor_msgs/image is a 1D vector where 1 pixel is given by 3 bytes
        // the RED, BLUE, and GREEN color information.
        // So, in each iteration, I'm checking three consecutive pixels/bytes
        for(int s=0; s<img.step; s+=3)
        {
            if((img.data[img.step*h + s] == white_pixel) && (img.data[img.step*h + s + 1] == white_pixel) && (img.data[img.step*h+ s + 2] == white_pixel))
            {             
                if(s < 0.33*img.step)
                {
                    left_count++;
                }
                else if(s >= 0.33*img.step && s < 0.66*img.step)
                {
                    mid_count++;
                }
                else
                {
                    right_count++;
                }
            }
        }
    }

    if((left_count == 0) && (mid_count == 0) && (right_count == 0))
    {
        drive_robot(0.0, 0.0); //stop
    }
    else
    {
        // Just finding the the max count
        if((left_count > right_count) && (left_count > mid_count))
        {
            drive_robot(0.33, 0.33);
        }
        else if((mid_count > left_count) && (mid_count > right_count))
        {
            drive_robot(0.33, 0.0);
        }
        else
        {
            drive_robot(0.33, -0.33);
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
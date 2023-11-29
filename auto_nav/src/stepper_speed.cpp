#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sstream>
#include <ros/console.h>

int speed_z, speed_x, sign_x, sign_z;
float stepZ, stepX;
double x, z;

// This callback takes velocity messages from cmd_vel topic
void calc_vel(const geometry_msgs::Twist &vel_msg)
{
    x = vel_msg.linear.x;
    z = vel_msg.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stepper_speed"); //set the node
    ros::NodeHandle n;                      // and the node handler

    ros::Subscriber vel_var = n.subscribe("cmd_vel", 1000, calc_vel); // Create the subscriber to cmd_vel topic

    ros::Publisher speed = n.advertise<geometry_msgs::Twist>("speed", 1000); // Create the publisher for speed
    ros::Publisher signX = n.advertise<std_msgs::Int16>("signX", 1000); // Create publisher for  X  (sign & steps)
    ros::Publisher signZ = n.advertise<std_msgs::Int16>("signZ", 1000); // Create publisher for  Z  (sign & steps)
   
    ros::Rate r(10); // The publish rate of messages
    ros::spinOnce();
    ROS_INFO("Converting DWA_planner_velocity -> arduino_velocity ");
    // START CALCULATING
    while (ros::ok())
    {
         // Create messages        
        geometry_msgs::Twist cmd_vel;
        std_msgs::Int16 xStep;
        std_msgs::Int16 zStep;

        // Check the sign for steps (Z)
        if (z < 0)
        {
            stepZ = 100;
        }
        else if (z > 0)
        {
            stepZ = -100;
        }
        else
        {
            stepZ = 0;
        }

        // Check the sign for steps (X)
        if (x > 0)
        {
            stepX = 100;
        }
        else if (x < 0)
        {
            stepX = -100;
        }
        else
        {
            stepX = 0;
        }

        // TRANSLATE THE LINEAR VELOCITIES TO STEPPER SPEED
        if ((x <= 0.06 && x > 0.05) || (x >= (-0.06) && x < (-0.05)))
        {
            speed_x = 600;
        }
        else if ((x <= 0.05 && x > 0.04) || (x >= (-0.05) && x < (-0.04)))
        {
            speed_x = 500;
        }
        else if ((x <= 0.04 && x > 0.03) || (x >= (-0.04) && x < (-0.03)))
        {
            speed_x = 400;
        }
        else if ((x <= 0.03 && x > 0.02) || (x >= (-0.03) && x < (-0.02)))
        {
            speed_x = 300;
        }
        else if ((x <= 0.02 && x > 0.01) || (x >= (-0.02) && x < (-0.01)))
        {
            speed_x = 200;
        }
        else if ((x <= 0.01 && x  > 0) || (x >= (-0.01) && x <0))
        {
            speed_x = 100;
        }

        // TRANSLATE THE ANGULAR VELOCITIES TO STEPPER SPEED
        if ((z <= 1.0 && z > 0.8) || (z >= (-1.2) && z < (-0.8)))
        {
            speed_z = 500;
        }
        else if ((z <= 0.8 && z > 0.6) || (z >= (-0.8) && z < (-0.6)))
        {
            speed_z = 400;
        }
        else if ((z <= 0.6 && z > 0.4) || (z >= (-0.6) && z < (-0.4)))
        {
            speed_z = 300;
        }
        else if ((z <= 0.4 && z > 0.2) || (z >= (-0.4) && z < (-0.2)))
        {
            speed_z = 200;
        }
        else if ((z <= 0.2 && z > 0) || (z >= (-0.2) && z < 0))
        {
            speed_z = 100;
        }

        // Pass the speed and sign values to messages
        cmd_vel.linear.x = speed_x;
        cmd_vel.angular.z = speed_z;
        xStep.data = stepX;
        zStep.data = stepZ;
        // ROS_INFO(" z is :  %d", stepX);
        // Publish the messages
        signX.publish(xStep);
        signZ.publish(zStep);
        speed.publish(cmd_vel);

        
        ros::spinOnce();
        r.sleep();
    }
}

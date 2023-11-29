#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>
#include <sstream>
#include <cmath>
#include <ros/console.h>


/*****************************************************************************************************************************
                                                VARIABLES
*****************************************************************************************************************************/
// CONST VARAIBLES
const float wheel_base = 270;  // in mm
const int circumference = 328; // in mm
const int TICKS_PER_MM = 4.9;  // ticks per mm
const float radius = 0.052;    // in m for angular velocity

//  TICK VARAIBLES
int lTick = 0; // left | right variables to store data  encoder topics
int rTick = 0;
int lTickPrev = 0; // old varaibles from left | right  encoders to calculate dl_tick | dr_tick
int rTickPrev = 0;
int dl_tick = 0; // calculation variables  of left | right ticks per cycle
int dr_tick = 0;

// DISTANCE VARAIBLES
double distance_left = 0; // distance traveled in mm from left | right wheel
double distance_right = 0;
double average_dist = 0; // average distance travelled (in mm) from both wheels combined
double x = 0; // x  , y  ,  z varaibles for odometry
double y = 0;
double z = 0;

// VELOCITY REALTED VARIABLES
double vx = 0; // linear velaocity and angular velocity
double vz = 0;
int dt = 0; //  dt of distance traveled. Variable that stores the data from topic time_lin which Arduino publishhes


/*****************************************************************************************************************************
                                                CALLBACKS
*****************************************************************************************************************************/

// LEFT ENCODER
void left_enc_cb(const std_msgs::Int32 &lTick_msg)
{
  lTick = lTick_msg.data; // assigns the data that have been published in topic "/left_enc"
}

// RIGHT ENCODER
void right_enc_cb(const std_msgs::Int32 &rTick_msg)
{
  rTick = rTick_msg.data; // assigns the data that have been published in topic "/right_enc"
}

// TIME PUBLIHED FROM ARDUINO
void dt_cb(const std_msgs::UInt16 &dt_ard)
{
  dt = dt_ard.data; // assigns the data extracted from arduino
  //ROS_INFO(" dt is :  %d", dt); // Debugging
}

/*****************************************************************************************************************************
                                                MAIN
*****************************************************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_calc"); // initiate node  "odom_calc"

  ros::NodeHandle n;                                                       // set node handler name
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000); // set the publisher for odom

  ros::Subscriber lTick_sub = n.subscribe("/left_enc", 1000, left_enc_cb); // set the subscribers for encoders
  ros::Subscriber rTick_sub = n.subscribe("/right_enc", 1000, right_enc_cb);
  ros::Subscriber dt_ssub = n.subscribe("/time_lin", 1000, dt_cb);

  tf::TransformBroadcaster odom_broadcaster; // Define broadcaster name
  ros::Rate r(10);                            // the rate data are exported


  // START CALCULATING
  while (ros::ok())
  {

    //  ticks measured in each cycle for left | right wheel
    dl_tick = lTick - lTickPrev;
    dr_tick = rTick - rTickPrev;
    // ROS_INFO(" is : %d, %d", dl_tick, dr_tick); // Debugging

    // store the old data from ticks to keep tracking the distance travelled at every cycle
    rTickPrev = rTick;
    lTickPrev = lTick;

    // calculation of distance for left and right
    distance_left = dl_tick / TICKS_PER_MM; // in mm
    distance_right = dr_tick / TICKS_PER_MM;
    // ROS_INFO(" L-R is : %lf %f", distance_left, distance_right); // Debugging

    average_dist = (distance_left + distance_right) / 2; // average distance travelled
    //ROS_INFO(" avg_dist is :  %f", average_dist); // Debugging

    // calculating velocities vx, vz
    vx = average_dist / dt;
    vz = vx / radius;

    // calculation of angle
    z += ((distance_right - distance_left) / wheel_base); // measured in radians

    z = atan2(sin(z), cos(z)); //  [-pi , pi]
    
    // calculating the x and y position that will be broadcasting with tf
    x += vx * cos(z);
    y += vx * sin(z);
    // ROS_INFO("X is : %lf", x); // Debugging
    // ROS_INFO("Z is : %lf", z); // Debugging


    // ROS_INFO("Vx is : %lf", vx);// Debugging
    // ROS_INFO("Vzis : %lf", vth);// Debugging 


    //-----> ODOMETRY <-----
    // Since all odometry is 6DOF we'll need a quaternion (orientation or rotations in 3D space) created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(z);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x / 1000;
    odom_trans.transform.translation.y = y / 1000;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x / 1000;
    odom.pose.pose.position.y = y / 1000;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vz;

    // publish the message
    odom_pub.publish(odom);

    r.sleep();
    ros::spinOnce();
  }
  
}

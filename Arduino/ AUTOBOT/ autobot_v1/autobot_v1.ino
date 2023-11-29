
/*********************************************
             LIBRARIES & MSGS
*********************************************/

//  Library for stepper motors  && encoders
#include <Stepper.h> 
#include <Encoder.h> 

// Libraries for ros and ros messages
#include <ros.h>    
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

/*********************************************
               VARIABLES
*********************************************/

// for stepper motor steps
int steps = 100;  // for stepper library
int step_x;       // forward-backwards
int step_z;       // left-right

// velocities
int linear = 0;  // The variables that store data from speedCB callback
int angular = 0;

// time
int currentMillis = 0;  //
int prevMillis = 0;
int old_time = 0;

/**************************************
               OBJECTS
**************************************/
Stepper FB(steps, 9, 11, 10, 12);    // Forward_backwards  
Stepper turn(steps, 11, 10, 12, 9);  //    Right  (Dir_pins : 11, 10 )--(Step_pins : 9, 12)
// Stepper turnLeft(steps , 10 , 9 , 11 , 12); //  alternative choice for turnning 

Encoder leftEnc(20, 21); // Encoder
Encoder rightEnc(2, 3);



/*********************************************
              CALLBACKS
*********************************************/

void speedCB(const geometry_msgs::Twist &vel) {  // Takes velocity values from /speed topic
// (which take values from cmd_vel topic and translate into stepper speed )
  linear = vel.linear.x;
  angular = vel.angular.z;
}

void signXCB(const std_msgs::Int16 &XSign) { // Takes  step sign which determines 
//the direction (forward-backwards) 
  step_x = XSign.data;
}

void signZCB(const std_msgs::Int16 &ZSign) {// Takes  step sign which determines
// the direction (left-right)
  step_z = ZSign.data;
}

/*********************************************
             ROS CONFIGURATIONS
*********************************************/
ros::NodeHandle nh;  // initiate Ros node
std_msgs::Int32 lTick_msg;  //set message variables 
std_msgs::Int32 rTick_msg;
std_msgs::UInt16 dt_msg;

/*********************************************
          SUBSCRIBERS AND PUBLISHER
*********************************************/
ros::Subscriber<std_msgs::Int16> sub_sign_z("signZ", signZCB);
ros::Subscriber<std_msgs::Int16> sub_sign_x("signX", signXCB);
ros::Subscriber<geometry_msgs::Twist> sub_speed("speed", speedCB);
ros::Publisher left_enc_pub("/left_enc", &lTick_msg);
ros::Publisher right_enc_pub("/right_enc", &rTick_msg);
ros::Publisher d_time("time_lin", &dt_msg);

/************************************************
                   SETUP
*************************************************/

void setup() {
  // Serial.begin(57600);
  nh.getHardware()->setBaud(57600);  // set the baud rate of serial comunication
  nh.initNode();                      // initiate    the node
  nh.subscribe(sub_speed);            // subscribe to cmd_vel topic

  nh.subscribe(sub_sign_z);  // subscribe to  signX, signZ for the direction of the robot
  nh.subscribe(sub_sign_x);

  nh.advertise(d_time);         // advertise time for odometry calculations
  nh.advertise(left_enc_pub);   // advertisers for left and
  nh.advertise(right_enc_pub);  // right encoders
}

/************************************************
                   LOOP
*************************************************/
void loop() {
  // if node is not connected set steps to 0 to stop any movement of the robot
  if (!nh.connected()) {
    FB.step(0);
    turn.step(0);
    nh.spinOnce();  // check if there is any event (in this case if nh is disconnected)
  }

  // while  node is connected execute the followings
  while (nh.connected()) {

    currentMillis = millis();

    if ((currentMillis - prevMillis) >= 50) {  //  loop

      if (step_x != 0) {  //  in case steps are not 0  :
        old_time = millis();
        FB.setSpeed(linear);                // Set linear velocity
        FB.step(step_x);                    // actuate steppers
        dt_msg.data = millis() - old_time;  //Calculate dt for odometry
      } else {                              // else if steps = 0 then velocity=0
        FB.step(0);                    // so it doesnt matter to measure dt
      }

      if (step_z != 0) {         // if steps for turning process are not 0 then :
        turn.setSpeed(angular);  // Set angular velocity
        turn.step(step_z);       // actuate steppers
      } else {
        turn.step(0);
      }

      lTick_msg.data = leftEnc.read();  // read encoders
      rTick_msg.data = rightEnc.read();

      left_enc_pub.publish(&lTick_msg);  // publish messages from encoders
      right_enc_pub.publish(&rTick_msg);

      d_time.publish(&dt_msg);  // publish dt for odometry calculations

      nh.spinOnce();  // process the events . Here it checks the callbacks
      // for velocity and step data updates
      prevMillis = currentMillis;  // set current time as previus 
    }
  }
}
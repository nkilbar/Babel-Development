/* Twist-Control
 * By Jonathan Daniel
 * 
 * This script uses the ROSSerial library to take a Twist message
 * that contains the speed for the Sabertooth and echoes the same speed
 * as a means of verification. We also use the SoftwareSerial library to move
 * Sabertooth communications away from the Hardware UART.
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SabertoothSimplified.h>
#include <SoftwareSerial.h>

#define MAX_SPEED 60

// Configure Sabertooth
SoftwareSerial SWSerial(0, 11);   // Make sure that the sabertooth serial pin is plugged into Digital 11.
SabertoothSimplified ST(SWSerial);        // Instantiate the Sabertooth with SoftwareSerial Port

// Configure ROS Node Handler and Twist Message
ros::NodeHandle nh;
geometry_msgs::Twist current_velocity;    // Return the current velocity for debugging purposes


// Callback function to update the velocity when we receive a new Twist message
void update_velocity(const geometry_msgs::Twist &cmd_vel){
  // Get the linear and angular speeds
  int goal_linear_speed = cmd_vel.linear.x;
  int goal_angular_speed = cmd_vel.angular.z;

  // Constrain our linear speed to be between 127 and -127
  if(goal_linear_speed > MAX_SPEED){
    nh.logwarn("Linear Speed greater than 127. Automatically lowered to 127");
    goal_linear_speed = MAX_SPEED;
  } else if(goal_linear_speed < -MAX_SPEED){
    nh.logwarn("Linear Speed less than -127. Automatically raised to -127");
    goal_linear_speed = -MAX_SPEED;
  }

  // Same approach with our angular speed
  if(goal_angular_speed > MAX_SPEED){
    nh.logwarn("Angular Speed greater than 127. Automatically lowered to 127");
    goal_angular_speed = MAX_SPEED;
  } else if(goal_angular_speed < -MAX_SPEED){
    nh.logwarn("Angular Speed less than -127. Automatically raised to -127");
    goal_angular_speed = -MAX_SPEED;
  }

  ST.drive(goal_linear_speed);
  ST.turn(goal_angular_speed);

  current_velocity.linear.x = goal_linear_speed;
  current_velocity.linear.y = 0;
  current_velocity.linear.z = 0;

  current_velocity.angular.x = 0;
  current_velocity.angular.y = 0;
  current_velocity.angular.z = goal_angular_speed;
}

// Setup ROS Publisher and Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("cmd_vel", &update_velocity );
ros::Publisher velocity_publisher("current_vel", &current_velocity);

void setup() {
  // Set Baud Rate to 9600 for Sabertooth and XBee Communications.
  Serial.begin(9600);
  nh.getHardware()->setBaud(9600);
  SWSerial.begin(9600);
  
  ST.drive(0);
  ST.turn(0);
  
  // Initialize the Node Handler
  nh.initNode();
  nh.advertise(velocity_publisher);
  nh.subscribe(cmd_vel_subscriber);
}

void loop() {
  // Get the speed values.
  velocity_publisher.publish(&current_velocity);
  nh.spinOnce();

  delay(100);

}

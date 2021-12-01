/*
    Instructions:
        1. Open three terminals 
        2. In one terminal run "roscore"
        3. In another terminal run "rosrun joy joy_node"
        4. Upload the Four_direction program in the Arduino IDE by pressing the right arrow button in the top left
        5. In the last terminal run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM# _baud:=9600" but use whatever ACM# is shown in the bottom right of the 
           Arduino IDE. Usually ACM0 or ACM1
        6. When turning everything off or uploading new code, press Ctrl^C in the rosserial terminal FIRST to stop the process.
           Failure to do so will cause issues.
        7. Also make sure the robot is not running when closing rosserial or uploading new code
*/
#define Serial SerialUSB
#define USE_USBCON 1

#include <SabertoothSimplified.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

SabertoothSimplified ST; 
ros::NodeHandle nh;

int drivVal = 0;
int turnVal = 0;
bool turnR = false;
bool turnL = false;

void joy_cb(const sensor_msgs::Joy& joy_node)
{
    drivVal = 40*joy_node.axes[1];
    turnVal = -30*joy_node.axes[2];
}

ros::Subscriber<sensor_msgs::Joy> joySub("/joy", &joy_cb);

void setup()
{         
  ST.drive(0);
  ST.turn(0);  
  SabertoothTXPinSerial.begin(9600); 
  
  nh.initNode();
  nh.subscribe(joySub);
}

void loop()
{
  nh.spinOnce();
  ST.drive(drivVal);
  ST.turn(turnVal);
    /*
  if(drivFwd) {
        ST.drive(40); //Value from -127 to 127, shared between motors if turn is 0
        ST.turn(0);
  } else if(drivBack){
        ST.drive(-40);
        ST.turn(0);
  } else if(turnR){
        ST.drive(0);
        ST.turn(30);
  } else if(turnL){
        ST.drive(0);
        ST.turn(-30);
  } else {
        ST.drive(0);
        ST.turn(0);
  }
  */
}

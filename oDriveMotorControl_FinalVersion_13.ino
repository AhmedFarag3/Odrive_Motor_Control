//*******************************************************************************
//*   Author  : Ahmed Farag                                                      *
//*   Date    : 01/04/2022                                                       *
//*   Version : V03                                                              *
//*******************************************************************************
// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

float Left_Wheel_Velocity_In_RPS_G ;
float Right_Wheel_Velocity_In_RPS_G;
int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
int axisnum = 0;



// Pins connection between oDrive and Tennsy 
// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// GND Odrive <-> GND Tennsy 
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy

// sensors input pins
int Sensor_One_Input_Pin   = 3;
int Sensor_Two_Input_Pin   = 4 ;
int Sensor_Three_Input_Pin = 19;
int Sensor_Four_Input_Pin  = 20;

// Communication with serial over serial1 
HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

// Create node handle to manage pub and sub
ros::NodeHandle nh;


void cmd_vel_cb(const geometry_msgs::Twist &cmdvel)
{
  float Angle_Speed  = cmdvel.angular.z;
  float Linear_Speed = cmdvel.linear.x;
  float Length = 0.33 ;
  float Wheel_Radius = 0.1;
  float Left_Wheel_Velocity = 0 ;
  float Right_Wheel_Velocity = 0;
  float Left_Wheel_Velocity_In_RPS = 0 ;
  float Right_Wheel_Velocity_In_RPS = 0;
  float Center_Of_Rotation = 0;  


  if(Angle_Speed != 0)
  {
   
   Center_Of_Rotation = Linear_Speed/Angle_Speed ;

  //Formula for converting the velocity cmd twist msg to motor velocity
  Left_Wheel_Velocity  = Linear_Speed + Angle_Speed  * (Center_Of_Rotation - Length/2);
  Right_Wheel_Velocity = Linear_Speed + Angle_Speed  * (Center_Of_Rotation + Length/2);
  
  Left_Wheel_Velocity_In_RPS  = Left_Wheel_Velocity  /  (2 * 3.14 * Wheel_Radius) ; 
  Right_Wheel_Velocity_In_RPS = Right_Wheel_Velocity / (2 * 3.14 * Wheel_Radius) ;
  
  Left_Wheel_Velocity_In_RPS_G = Left_Wheel_Velocity_In_RPS ;
  Right_Wheel_Velocity_In_RPS_G = Right_Wheel_Velocity_In_RPS;

  }
  else
  { 
   Left_Wheel_Velocity_In_RPS = Right_Wheel_Velocity_In_RPS = Linear_Speed /  (2 * 3.14 * Wheel_Radius) ; 
   Left_Wheel_Velocity_In_RPS_G   = Left_Wheel_Velocity_In_RPS ;
   Right_Wheel_Velocity_In_RPS_G  = Right_Wheel_Velocity_In_RPS;

  }
    //debugging
  nh.loginfo((String("V_l = ") + String(Left_Wheel_Velocity_In_RPS).c_str()).c_str());
  nh.loginfo("-----------------------------------------------------------------------------------");
  nh.loginfo((String("V_r = ") + String(Right_Wheel_Velocity_In_RPS).c_str()).c_str());
}


// Creating a ros subscriber for cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

  
void setup() {

  // sets the digital pin 7 as input
  pinMode(Sensor_One_Input_Pin, INPUT);    
  pinMode(Sensor_Two_Input_Pin, INPUT);   
  pinMode(Sensor_Three_Input_Pin, INPUT);   
  pinMode(Sensor_Four_Input_Pin, INPUT);   
  //Odrive BaudRate 115200
  // Start serial communication with oDrive on serial 1
  odrive_serial.begin(115200);
  

  // Initialization node function
  nh.initNode();
  //Subscriber node for cmd vel topic
  nh.subscribe(cmd_vel_sub);

  
  //start serial with ROS
  Serial.begin(115000);

}

void loop() {

  odrive.SetVelocity(0,Right_Wheel_Velocity_In_RPS_G);
  odrive.SetVelocity(1,Left_Wheel_Velocity_In_RPS_G);
  delay(5);



//  if((digitalRead(Sensor_One_Input_Pin)==1) || (digitalRead(Sensor_Two_Input_Pin)==1)) 
//  {
//    Serial.println("Object Detected");
//    odrive.SetVelocity(0,0);
//  }
//  else if((digitalRead(Sensor_One_Input_Pin)==0) || (digitalRead(Sensor_Two_Input_Pin)==0))
//  {
//    Serial.println("No Object");
//    
//  }
    
  nh.spinOnce();
  delay(5);
}

#include <Arduino.h>
//*******************************************************************************
//*   Author  : Ahmed Farag                                                      *
//*   Date    : 01/04/2022                                                       *
//*   Version : V03                                                              *
//*******************************************************************************
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Pins connection between oDrive and Tennsy
// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX 1
// pin 1: TX - connect to ODrive RX 2
// GND Odrive <-> GND Tennsy
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy


// Printing with stream operator helper functions
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

float Left_Wheel_Velocity_In_RPS_G;
float Right_Wheel_Velocity_In_RPS_G;

// sensors input pins
int Sensor_One_Input_Pin = 3;
int Sensor_Two_Input_Pin = 4;
int Sensor_Three_Input_Pin = 19;
int Sensor_Four_Input_Pin = 20;

// Communication with serial over serial1
HardwareSerial &odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

// Create node handle to manage pub and sub
ros::NodeHandle nh;

void cmd_vel_cb(const geometry_msgs::Twist &cmdvel)
{
  float Angle_Speed = cmdvel.angular.z;
  float Linear_Speed = cmdvel.linear.x;
  float Length = 0.33;
  float Wheel_Radius = 0.1;
  float Left_Wheel_Velocity = 0;
  float Right_Wheel_Velocity = 0;
  float Left_Wheel_Velocity_In_RPS = 0;
  float Right_Wheel_Velocity_In_RPS = 0;
  float Center_Of_Rotation = 0;

  if (Angle_Speed != 0)
  {

    Center_Of_Rotation = Linear_Speed / Angle_Speed;

    // Formula for converting the velocity cmd twist msg to motor velocity
    Left_Wheel_Velocity = Linear_Speed + Angle_Speed * (Center_Of_Rotation - Length / 2);
    Right_Wheel_Velocity = Linear_Speed + Angle_Speed * (Center_Of_Rotation + Length / 2);

    Left_Wheel_Velocity_In_RPS = Left_Wheel_Velocity / (2 * 3.14 * Wheel_Radius);
    Right_Wheel_Velocity_In_RPS = Right_Wheel_Velocity / (2 * 3.14 * Wheel_Radius);

    Left_Wheel_Velocity_In_RPS_G = Left_Wheel_Velocity_In_RPS;
    Right_Wheel_Velocity_In_RPS_G = Right_Wheel_Velocity_In_RPS;
  }
  else
  {
    Left_Wheel_Velocity_In_RPS = Right_Wheel_Velocity_In_RPS = Linear_Speed / (2 * 3.14 * Wheel_Radius);
    Left_Wheel_Velocity_In_RPS_G = Left_Wheel_Velocity_In_RPS;
    Right_Wheel_Velocity_In_RPS_G = Right_Wheel_Velocity_In_RPS;
  }
  nh.loginfo("Received");
}

// Creating a ros subscriber for cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

void setup()
{

  // sets the digital pin 7 as input
  //  pinMode(Sensor_One_Input_Pin, INPUT);
  //  pinMode(Sensor_Two_Input_Pin, INPUT);
  //  pinMode(Sensor_Three_Input_Pin, INPUT);
  //  pinMode(Sensor_Four_Input_Pin, INPUT);
  // Odrive BaudRate 115200
  // Start serial communication with oDrive on serial 1
  odrive_serial.begin(115200);

  // Initialization node function
  nh.initNode();
  // Subscriber node for cmd vel topic
  nh.subscribe(cmd_vel_sub);

  // start serial with ROS
  Serial.begin(115000);
}

void loop()
{

  float PosM0 = odrive.GetPosition(0);
  float PosM1 = odrive.GetPosition(1);

  float VelM0 = odrive.GetVelocity(0);
  float VelM1 = odrive.GetVelocity(1);

  float diff = VelM0 - VelM1;
  // Read Vel R= -0.24
  // Read Vel L= -0.06
  if (diff < -0.05)
  {
    float ratio = VelM1 / VelM0;
    float IncSpeed = 1 - ratio;
    Left_Wheel_Velocity_In_RPS_G = Left_Wheel_Velocity_In_RPS_G * (1 + IncSpeed);
  }

  else if (diff > 0.05)
  {
    float ratio = VelM0 / VelM1;
    float IncSpeed = 1 - ratio;
    Right_Wheel_Velocity_In_RPS_G = Right_Wheel_Velocity_In_RPS_G * (1 + IncSpeed);
  }
  else
  {
    Left_Wheel_Velocity_In_RPS_G = Left_Wheel_Velocity_In_RPS_G;
    Right_Wheel_Velocity_In_RPS_G = Right_Wheel_Velocity_In_RPS_G;
  }

  Right_Wheel_Velocity_In_RPS_G = constrain(Right_Wheel_Velocity_In_RPS_G, -0.3, 0);

  Left_Wheel_Velocity_In_RPS_G = constrain(Left_Wheel_Velocity_In_RPS_G, -0.3, 0);

  // debugging
  nh.loginfo((String("V_l = ") + String(Left_Wheel_Velocity_In_RPS_G).c_str()).c_str());
  nh.loginfo((String("V_r = ") + String(Right_Wheel_Velocity_In_RPS_G).c_str()).c_str());
  nh.loginfo("-----------------------------------------------------------------------------------");

  nh.loginfo((String("Read Vel R= ") + String(VelM0).c_str()).c_str());
  nh.loginfo((String("Read Vel L= ") + String(VelM1).c_str()).c_str());
  nh.loginfo("-----------------------------------------------------------------------------------");

  nh.loginfo((String("Read Pos R= ") + String(PosM0).c_str()).c_str());
  nh.loginfo((String("Read Pos L= ") + String(PosM1).c_str()).c_str());
  nh.loginfo("-----------------------------------------------------------------------------------");

  odrive.SetVelocity(0, Right_Wheel_Velocity_In_RPS_G);
  odrive.SetVelocity(1, Left_Wheel_Velocity_In_RPS_G);
  //  delay(5);

  nh.spinOnce();
  delay(1);
}
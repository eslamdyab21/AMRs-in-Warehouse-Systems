
#include "IMU6050.h"
#include "z_I2Cdev.h"
#define __STM32F1__
// Ros includes
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Byte.h>

// Sensors Pins
#define voltage_sensor PB0 
#define current_sensor PB1 


ros::NodeHandle_<ArduinoHardware, 1, 3, 2048, 2048> nh;

const int Master_address=0x01;

const unsigned long imu_timer = 50;
const unsigned long debugger_timer = 1000;


float P;
int L;
float *ypr; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long timeNow;
unsigned long prevTimeImu = millis();
unsigned long prevTimeDebugger = millis();
float yaw_angle;
IMU6050 imu;

// Publisher for yaw 
std_msgs::Float32 yaw;
ros::Publisher pub_yaw("yaw", &yaw);

std_msgs::Float32 voltage;
ros::Publisher pub_voltage("voltage", &voltage);

std_msgs::Float32 current;
ros::Publisher pub_current("current", &current);
void setup()
{
  Wire.begin(Master_address);

  Serial.begin(9600);

  delay(500);
  imu.intilaize();
  delay(500);
  nh.initNode();
  nh.advertise(pub_yaw);
  nh.advertise(pub_voltage);
  nh.advertise(pub_current);
 
}


void loop()
{
  timeNow = millis();
  
    if (timeNow - prevTimeImu > imu_timer)
    {
    prevTimeImu = timeNow;
    ypr = imu.get_yaw_pitch_roll();  // Roll is flipped with Pitch
    yaw_angle = ypr[0] * 180 / M_PI;
 
    yaw.data = yaw_angle;
    pub_yaw.publish(&yaw); 
    Serial.println(yaw_angle);
  }

   
  if (timeNow - prevTimeDebugger > debugger_timer)
  {
    prevTimeDebugger = timeNow;

    if (!imu.isConnected())
    {
      //Serial.println("MPU6050 connection failed");
      Wire.begin(Master_address);
    }

  }
   nh.spinOnce();  
}

#define __STM32F1__
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Byte.h>
#include "PID.h"

PID motorLeft(0);
PID motorRight(0);
PID yawPID(0);
ros::NodeHandle_<ArduinoHardware, 4, 1, 2048, 2048> nh;

// left motor pins on cytron
#define dir_left_motor PA5 
#define pwm_left_motor PA6 

// right motor pins on cytron
#define dir_right_motor PA0 
#define pwm_right_motor PA1 

// Encoder pins for right motor
#define pinA_encoder_right PA9 
#define pinB_encoder_right PA8 

// Encoder pins for left motor
#define pinA_encoder_left PB10 
#define pinB_encoder_left PB11 

/*---------------- Variables declarations ----------------*/
// counts for  encoders
int count_left = 0, count_right = 0, count_lift = 0; 

// Speeds for motors 
float speed_left = 0, speed_right = 0;

// IMU variables 
float yaw_angle = 0;

// States for encoders 
int current_state_l = 0, current_state_r = 0, current_state_lift = 0;

// direction of encoders 
String encdir_l ="", encdir_r ="", encdir_lift ="";  

// Speeds variables and damping factor
float y = 0, m = 0, epsi = 0.8, y_left = 0, y_right = 0, m_rot = 0;

// PWMs for motors
float pwm_left = 0, pwm_right = 0;

// timers variables
double prev_time = 0, speed_timer = 50;
double action_timer = 10, last_action = 0;
double feedback_timer = 1000, prev_feedback_timer = 0;

// PID constants
double kp = 0, ki = 0.0, kd = 0.0;

// Speeds feedback
short int speeds_temp[2] = {0, 0};

/*---------------- Interrubts functions ----------------*/

void funcL(){
   current_state_l = digitalRead(pinA_encoder_left);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_left) != current_state_l) { 
     count_left--;
     
     
   } else {
     // Encoder is rotating counterclockwise
     count_left++;
     
   }

}


void funcR(){
   current_state_r = digitalRead(pinA_encoder_right);
   
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_right) != current_state_r) { 
     count_right--;
    
     
   } else {
     // Encoder is rotating counterclockwise
     count_right++;
    
   }

}
/*---------------- Callback Functions For ROS ----------------*/
void setSpeeds(const std_msgs::Int16MultiArray& speeds)
{
  short int *got = speeds.data;
  y = got[0];
  m = got[1];
  debugger(y,"Y: ");
  debugger(m,"M: ");
}

void setPid(const std_msgs::Float32MultiArray& constants)
{
  kp = constants.data[0];
  ki = constants.data[1];
  kd = constants.data[2];
//  motorLeft.set_constants(kp, ki, kd);
//  motorRight.set_constants(kp, ki,kd);
  yawPID.set_constants(kp, ki, kd);
  debugger(kp, "kP: ");
  debugger(ki, "kI: ");
  debugger(kd, "kD: ");
}

void set_setPoint(const std_msgs::Int16& setPoint)
{
//  y = setPoint.data;
  debugger(setPoint.data, "set point: ");
//  motorLeft.set_setpoint(setPoint.data);
//  motorRight.set_setpoint(setPoint.data);
  yawPID.set_setpoint(setPoint.data);  

}

void setYaw(const std_msgs::Float32& got)
{
  yaw_angle = got.data;


}
// Publisher and subscribers for feedback
std_msgs::Int16MultiArray feedback;
ros::Publisher pub_feedback("feedback", &feedback);

ros::Subscriber<std_msgs::Int16MultiArray> sub_speeds("ros_robot_move_stm", &setSpeeds );
ros::Subscriber<std_msgs::Int16> sub_setPoint("setPoint", &set_setPoint );
ros::Subscriber<std_msgs::Float32MultiArray> sub_pid("pid", &setPid );
ros::Subscriber<std_msgs::Float32> sub_yaw("yaw", &setYaw );
/*---------------- Debugger for ROS ----------------*/

void debugger(float data, String topic)
{
  topic+= String(data);
  nh.loginfo(topic.c_str());
}

void setup() {
  // put your setup code here, to run once:
  pinMode(dir_left_motor, OUTPUT);
  pinMode(pwm_left_motor, OUTPUT);
  pinMode(dir_right_motor, OUTPUT);
  pinMode(pwm_right_motor, OUTPUT);
  
  pinMode(pinA_encoder_right,INPUT);
  pinMode(pinB_encoder_right,INPUT);
  pinMode(pinA_encoder_left,INPUT);
  pinMode(pinB_encoder_left,INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA_encoder_left), funcL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_right), funcR,CHANGE);
  
//  Serial.begin(9600);
  // Ros nodes declarations
  nh.initNode();
  nh.subscribe(sub_speeds);
  nh.subscribe(sub_pid);
  nh.subscribe(sub_setPoint);
  nh.advertise(pub_feedback);
  nh.subscribe(sub_yaw);
  
  // PID constants and constraints
//  motorLeft.set_constants(kp, ki,kd);
//  motorLeft.set_constrains(-255, 255);
//  motorRight.set_constants(kp, ki,kd);
//  motorRight.set_constrains(-255, 255);
//  motorLeft.set_setpoint(y);
//  motorRight.set_setpoint(y);
    yawPID.set_constants(kp, ki,kd);
    yawPID.set_constrains(-255, 255);
}

void loop() {

   // Calculating linear speeds for motors
  if ((millis()-prev_time) > speed_timer)
  {
   
    speed_left = (count_left/(450*(speed_timer/1000)))*60;
    count_left = 0;

    speed_right = (count_right/(450*(speed_timer/1000)))*60;
    
    count_right = 0;  
    
    // Send speeds to feedback
    speeds_temp[0] = round(speed_left);
    speeds_temp[1] = round(speed_right);
      
//    y_right = motorRight.calc(speed_right);
//    y_left = motorLeft.calc(speed_left);
//    pwm_left = pwm_left*epsi + (y_left+m)*(1 - epsi);
//    pwm_right = pwm_right*epsi + (y_right-m)*(1 - epsi);
//   
//   Serial.print("speed_left:");
//   Serial.print(speed_left);
//   Serial.print(",");
//   Serial.print("speed_right:");
//   Serial.print(speed_right);
//   Serial.print(",");
//   Serial.print("set_point:");
//   Serial.println(y);
   
    
    feedback.data_length = 2;
    feedback.data = speeds_temp;
    pub_feedback.publish(&feedback);
    prev_time = millis();
  }
  
    
    
    
    
  if ((millis() - last_action) > action_timer)
  {
    
    m_rot = yawPID.calc(yaw_angle);
  // Sending Speeds to be applied
    pwm_left = pwm_left*epsi + (y+m_rot)*(1 - epsi);
    pwm_right = pwm_right*epsi + (y-m_rot)*(1 - epsi);
    if (pwm_left >= 0)
    {
      analogWrite(pwm_left_motor,abs(pwm_left));
      digitalWrite(dir_left_motor, HIGH);
    }
    else
    {
      analogWrite(pwm_left_motor,abs(pwm_left));
      digitalWrite(dir_left_motor, LOW);
    }
    if (pwm_right >= 0)
    {
      analogWrite(pwm_right_motor,abs(pwm_right));
      digitalWrite(dir_right_motor, HIGH);
    }
    else
    {
      analogWrite(pwm_right_motor,abs(pwm_right));
      digitalWrite(dir_right_motor, LOW);
    }
    last_action = millis();
   
  }
  nh.spinOnce();
    

}

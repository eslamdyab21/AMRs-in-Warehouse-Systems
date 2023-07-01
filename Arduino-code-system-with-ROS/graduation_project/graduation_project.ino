#define __STM32F1__
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Byte.h>
#include "PID.h"

PID yawPID(1);
//PID motorLeft(0);
//PID motorRight(0);
// REALLY IMPORTANT TO DECREASE INPUT AND OUTPUT BUFFER SIZE, here set to 100 bytes each
ros::NodeHandle_<ArduinoHardware, 6, 2, 2048, 2048> nh;


// left motor pins on cytron
#define dir_left_motor PA5 
#define pwm_left_motor PA6 

// right motor pins on cytron
#define dir_right_motor PA0 
#define pwm_right_motor PA1 

// Encoder pins for right motor
#define pinA_encoder_right PA9 
#define pinB_encoder_right PA8 

// Encoder pins for lifting motor 
#define pinA_encoder_lifting PB9 
#define pinB_encoder_lifting PB8 

// Encoder pins for left motor
#define pinA_encoder_left PB10 
#define pinB_encoder_left PB11 

// LM298 pins for lifting motor
#define pin_move_lifitng PB12 
#define pinA_lifting PB13 
#define pinB_lifting PB14

#define pi 3.14159265359
/*---------------- Variables declarations ----------------*/

// counts for  encoders
int count_left = 0, count_right = 0, count_lift = 0; 

// Speeds for motors 
float speed_left = 0, speed_right = 0, RPS = 0;
double rot_dist_left = 0, rot_dist_right = 0, R = 31, ang = 0;
// IMU variables 
float yaw_angle = 0, desired_yaw = 0;

// States for encoders 
int current_state_l = 0, current_state_r = 0, current_state_lift = 0, m_rot_desired = 40;


// Speeds variables and damping factor
float y = 0, m = 0, m_rot = 0, epsi = 0.8, y_left = 0, y_right = 0;

// PWMs for motors
float pwm_left = 0.0, pwm_right = 0.0;

// timers variables
double prev_time = 0, speed_timer = 50;
double action_timer = 10, last_action = 0;
double feedback_timer = 1000, prev_feedback_timer = 0;
double distance_right = 0.0, distance_left = 0.0, prev_distance = 0.0;
int qr_separation = 50;
// PID constants
float kp = 4, ki = 2, kd = 0.1;

// Speeds feedback
short int speeds_temp[2] = {0, 0};
/*---------------- Publishers for ros ----------------*/

// Publisher for feedback
std_msgs::Int16MultiArray feedback;
ros::Publisher pub_feedback("feedback", &feedback);

// Publisher for callback from stm
std_msgs::Int8 stm_callback;
ros::Publisher pub_stm_callback("ros_robot_moved_feedback_stm_callback", &stm_callback);
/*---------------- Interrubts functions ----------------*/

void funcL(){
   current_state_l = digitalRead(pinA_encoder_left);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_left) != current_state_l) 
   { 
     count_left--;
     
   } 
   else 
   {
     // Encoder is rotating counterclockwise
     count_left++;
   }

}

void funcLift(){
   current_state_lift = digitalRead(pinA_encoder_lifting);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_lifting) != current_state_lift) 
   { 
     count_lift++;
   } 
   else 
   {
     // Encoder is rotating counterclockwise
     count_lift --;
     
   }

}

void funcR(){
   current_state_r = digitalRead(pinA_encoder_right);
   
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_right) != current_state_r) 
   { 
     count_right--;
     
   } 
   else 
   {
     // Encoder is rotating counterclockwise
     count_right++;
   }

}

/*---------------- Callback Functions For ROS ----------------*/

void setSpeeds(const std_msgs::Int16MultiArray& speeds)
{
  short int *got = speeds.data;
  y = (300*got[0])/100;
  m = got[1];
//  if(abs(y) > 0)
//  {
//  motorLeft.set_setpoint(y);
//  motorRight.set_setpoint(y);
//  motorLeft.set_constants(kp, ki, kd);
//  motorRight.set_constants(kp, ki,kd);
//  }
  if(abs(y) > 0)
  {
  yawPID.set_setpoint(yaw_angle);
  yawPID.set_constants(kp, ki,kd);
  }
  debugger(y, "Speed: ");
  debugger(m, "Moment: ");
}
void setSpeedRot(const std_msgs::Int16& speeds)
{
  short int got = speeds.data;
  m_rot_desired = got;
  debugger(m_rot_desired, "rotation speed: ");
}
void setYaw(const std_msgs::Float32& yaw)
{
  yaw_angle = yaw.data;
  //debugger(yaw_angle, "Yaw: ");

}

void setQr(const std_msgs::Float32& qr)
{
  qr_separation = qr.data;
  debugger(qr_separation, "qr_separation: ");

}

void setSpeedsLR(const std_msgs::Int16MultiArray& speeds)
{
  short int *got = speeds.data;
  y_left = (255*got[0])/100;
  y_right = (255*got[1])/100;

  debugger(y_left, "y_left: ");
  debugger(y_right, "y_right: ");
}
void setPid(const std_msgs::Float32MultiArray& constants)
{
  kp = constants.data[0];
  ki = constants.data[1];
  kd = constants.data[2];
//  motorLeft.set_constants(kp, ki, kd);
//  motorRight.set_constants(kp, ki,kd);
  yawPID.set_constants(kp, ki,kd);
  debugger(kp, "kP: ");
  debugger(ki, "kI: ");
  debugger(kd, "kD: ");
}
ros::Subscriber<std_msgs::Int16MultiArray> sub_speeds("ros_robot_move_stm", &setSpeeds );
ros::Subscriber<std_msgs::Int16> sub_rot_speeds("rotate_speed", &setSpeedRot );
ros::Subscriber<std_msgs::Float32> sub_yaw("yaw", &setYaw );
ros::Subscriber<std_msgs::Float32> sub_qr("qr_separation", &setQr );
ros::Subscriber<std_msgs::Int16MultiArray> sub_speeds_l_r("speeds", &setSpeedsLR );
ros::Subscriber<std_msgs::Float32MultiArray> sub_pid("pid", &setPid );
/*---------------- Check for rotation function ----------------*/


void check_rotation()
{
  // Algorithm for rotation clockwise
  if (m == -1)
  {
    desired_yaw = yaw_angle + 90;
    if (desired_yaw > 180)
    {
      desired_yaw = desired_yaw - 360;
    }
    y = 0;
    m_rot = m_rot_desired;
    count_left = 0;
    count_right = 0;
    yawPID.set_constants(0, 0,0);
    debugger(desired_yaw, "desired yaw: ");
    while(1)
    {
      
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
      if (yaw_angle >= desired_yaw)
      {
        analogWrite(pwm_left_motor,0);
        analogWrite(pwm_right_motor,0);
        
        yawPID.set_constants(kp, ki,kd);
        yawPID.set_setpoint(desired_yaw);
        
        distance_right = 0.0;
        distance_left = 0.0;
        m = 0;
        m_rot = 0;
        y = 0;
        
        stm_callback.data = 1;
        pub_stm_callback.publish(&stm_callback);
        break;
      }
      nh.spinOnce();
    }
    count_left = 0;
    count_right = 0;
    
    
  }
  // Algorithm for rotation counterclockwise
  if (m == 1)
  {
    count_left = 0;
    count_right = 0;
    yawPID.set_constants(0, 0,0);
    desired_yaw = yaw_angle - 90;
    if (desired_yaw < -180)
    {
      desired_yaw = desired_yaw + 360;
    }
    y = 0;
    m_rot = -m_rot_desired;
    debugger(desired_yaw, "desired yaw: ");
    while(1)
    {
      
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
      if (yaw_angle <= desired_yaw)
      {
        
        analogWrite(pwm_left_motor,0);
        analogWrite(pwm_right_motor,0);
//        motorLeft.set_constants(0, 0,0);
//        motorRight.set_constants(0, 0,0);
        yawPID.set_constants(kp, ki,kd);
        yawPID.set_setpoint(desired_yaw);
        distance_right = 0.0;
        distance_left = 0.0;
        m = 0;
        m_rot = 0;
        y = 0;
        
        stm_callback.data = 1;
        pub_stm_callback.publish(&stm_callback);
        break;
      }
     nh.spinOnce(); 
    }
    count_left = 0;
    count_right = 0;
  }
//if (m==-1)
//    {
//        count_left = 0;
//        count_right = 0;
//        while(1)
//        {
//          y = 0;
//          m_rot = 45;
//          ang = abs(count_left*0.3733);
//          pwm_left = pwm_left*epsi + (y+m_rot)*(1 - epsi);
//          pwm_right = pwm_right*epsi + (y-m_rot)*(1 - epsi);
//          if (pwm_left >= 0)
//          {
//            analogWrite(pwm_left_motor,abs(pwm_left));
//            digitalWrite(dir_left_motor, HIGH);
//          }
//          else
//          {
//            analogWrite(pwm_left_motor,abs(pwm_left));
//            digitalWrite(dir_left_motor, LOW);
//          }
//          if (pwm_right >= 0)
//          {
//            analogWrite(pwm_right_motor,abs(pwm_right));
//            digitalWrite(dir_right_motor, HIGH);
//          }
//          else
//          {
//            analogWrite(pwm_right_motor,abs(pwm_right));
//            digitalWrite(dir_right_motor, LOW);
//          }
//          if(ang >= 90)
//          {
//            analogWrite(pwm_left_motor,0);
//            analogWrite(pwm_right_motor,0);
//            motorLeft.set_constants(0, 0,0);
//            motorRight.set_constants(0, 0,0);
//            distance_right = 0.0;
//            distance_left = 0.0;
//            m = 0;
//            m_rot = 0;
//            y = 0;
//            
//            stm_callback.data = 1;
//            pub_stm_callback.publish(&stm_callback);
//            break;
//          }
//          nh.spinOnce();
//        }
//        count_left = 0;
//        count_right = 0;
//        debugger(ang, "Angle ");
//        
//    }
//    if (m==1)
//    {
//
//        count_left = 0;
//        count_right = 0;
// 
//        while(1)
//        {
//
//          y = 0;
//          m_rot = -45;
//          ang = abs(count_right*0.3733);
//          pwm_left = pwm_left*epsi + (y+m_rot)*(1 - epsi);
//          pwm_right = pwm_right*epsi + (y-m_rot)*(1 - epsi);
//          if (pwm_left >= 0)
//          {
//            analogWrite(pwm_left_motor,abs(pwm_left));
//            digitalWrite(dir_left_motor, HIGH);
//          }
//          else
//          {
//            analogWrite(pwm_left_motor,abs(pwm_left));
//            digitalWrite(dir_left_motor, LOW);
//          }
//          if (pwm_right >= 0)
//          {
//            analogWrite(pwm_right_motor,abs(pwm_right));
//            digitalWrite(dir_right_motor, HIGH);
//          }
//          else
//          {
//            analogWrite(pwm_right_motor,abs(pwm_right));
//            digitalWrite(dir_right_motor, LOW);
//          }
//          if(ang >= 90)
//          {
//            analogWrite(pwm_left_motor,0);
//            analogWrite(pwm_right_motor,0);
//            motorLeft.set_constants(0, 0,0);
//            motorRight.set_constants(0, 0,0);
//            m = 0;
//            m_rot = 0;
//            y = 0;
//            distance_right = 0.0;
//            distance_left = 0.0;
//            stm_callback.data = 1;
//            pub_stm_callback.publish(&stm_callback);
//            break;
//          }
//          nh.spinOnce();
//        }
//        count_left = 0;
//        count_right = 0;
//        debugger(ang, "Angle ");
//        
//    }
//  

}

void debugger(float data, String topic)
{
  topic+= String(data);
  nh.loginfo(topic.c_str());
}

void setup() {
  Serial.begin(9600);
  // Pin mode declarations
  pinMode(dir_left_motor, OUTPUT);
  pinMode(pwm_left_motor, OUTPUT);
  pinMode(dir_right_motor, OUTPUT);
  pinMode(pwm_right_motor, OUTPUT);
  pinMode(pin_move_lifitng, OUTPUT);
  pinMode(pinA_lifting, OUTPUT);
  pinMode(pinB_lifting, OUTPUT);
  pinMode(pinA_encoder_right,INPUT);
  pinMode(pinB_encoder_right,INPUT);
  pinMode(pinA_encoder_left,INPUT);
  pinMode(pinB_encoder_left,INPUT);
  pinMode(pinA_encoder_lifting,INPUT);
  pinMode(pinB_encoder_lifting,INPUT);
  //delay(100);
  //ISR for encoders declarations
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_left), funcL,RISING);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_lifting), funcLift,RISING);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_right), funcR,RISING);
  

  // Ros nodes declarations
  nh.initNode();
  nh.subscribe(sub_speeds);
  nh.subscribe(sub_yaw);
  nh.subscribe(sub_qr);
  nh.subscribe(sub_speeds_l_r);
  nh.subscribe(sub_rot_speeds);
  nh.subscribe(sub_pid);

  nh.advertise(pub_feedback);
  nh.advertise(pub_stm_callback);

  // PID constants and constraints
//  motorLeft.set_constants(kp, ki,kd);
//  motorLeft.set_constrains(-255, 255);
//  motorRight.set_constants(kp, ki,kd);
//  motorRight.set_constrains(-255, 255);
//  motorLeft.set_setpoint(y);
//  motorRight.set_setpoint(y);
    yawPID.set_constants(0,0,0);
    yawPID.set_constrains(-255, 255);
    yawPID.set_setpoint(yaw_angle);
}

void loop() {
  // Calculating linear speeds for motors
  if ((millis()-prev_time) > speed_timer)
  {
    
    distance_left = distance_left + abs(count_left)*0.0955;
    distance_right = distance_right + abs(count_right)*0.0955;
    speed_left = ((count_left/(230*(speed_timer/1000))))*60;
    count_left = 0;

    speed_right = ((count_right/(230*(speed_timer/1000))))*60;
    count_right = 0;
    // Sending Speeds to be applied
//    y_right = motorRight.calc(speed_right);
//    y_left = motorLeft.calc(speed_left);
    

    if ((distance_left >= qr_separation) && (m == 0))
    {
      if((distance_right >= qr_separation))
      {
      y = 0;
      m_rot = 0;
      analogWrite(pwm_left_motor,0);
      analogWrite(pwm_right_motor,0);
//      motorLeft.set_constants(0, 0,0);
//      motorRight.set_constants(0, 0,0);
      yawPID.set_constants(0, 0,0);
      debugger(distance_left,"distance Left: ");
      debugger(distance_right,"distance Right: ");
      distance_left = 0;
      distance_right = 0;
      stm_callback.data = 1;
      pub_stm_callback.publish(&stm_callback);
      }
    }
    
    prev_time = millis();
    // Send speeds to feedback
    speeds_temp[0] = round(speed_left);   
    speeds_temp[1] = round(speed_right);
  }
  check_rotation();
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
  
  // Sending feedback values of motors using ROS
  if ((millis() - prev_feedback_timer) > feedback_timer)
  {

    
    feedback.data_length = 2;
    feedback.data = speeds_temp;
    
    pub_feedback.publish(&feedback);
    prev_feedback_timer = millis();
  }
  
  nh.spinOnce();
}

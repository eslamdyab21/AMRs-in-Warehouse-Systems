#define __STM32F1__
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Byte.h>


// REALLY IMPORTANT TO DECREASE INPUT AND OUTPUT BUFFER SIZE, here set to 100 bytes each
ros::NodeHandle_<ArduinoHardware, 1, 4, 2048, 2048> nh;


// left motor pins on cytron
#define dir_left_motor PA5 
#define pwm_left_motor PA6 

// right motor pins on cytron
#define dir_right_motor PA2 
#define pwm_right_motor PA3 

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

// Sensors Pins
#define voltage_sensor PB0 
#define current_sensor PB1 
// IMU pins 

#define SCL_pin PB6 
#define SDA_pin PB7 


/*---------------- Variables declarations ----------------*/

// counts for  encoders
int count_left = 0, count_right = 0, count_lift = 0; 

// Speeds for motors 
float speed_left = 0, speed_right = 0;

// IMU variables 
float yaw_angle = 0, pitch_angle = 0, roll_angle = 0;

// States for encoders 
int current_state_l = 0, current_state_r = 0, current_state_lift = 0;

// direction of encoders 
String encdir_l ="", encdir_r ="", encdir_lift ="";  

// Speeds variables and damping factor
float y = 0, m = 0, epsi = 0.8;

// PWMs for motors
int pwm_left = 0, pwm_right = 0;

// timers variables
double prev_time = 0, speed_timer = 1000;
double action_timer = 10, last_action = 0;
double imu_timer = 50, prev_imu_timer = 0;
double feedback_timer = 1000, prev_feedback_timer = 0;
  
/*---------------- Publishers for ros ----------------*/

// Publisher for feedback
std_msgs::Int16MultiArray feedback;
ros::Publisher pub_feedback("feedback", &feedback);
// Publisher for yaw 
std_msgs::Float32 yaw;
ros::Publisher pub_yaw("yaw", &yaw);
// Publisher for pitch
std_msgs::Float32 pitch;
ros::Publisher pub_pitch("pitch", &pitch);
// Publisher for roll
std_msgs::Float32 roll;
ros::Publisher pub_roll("roll", &roll);

/*---------------- Interrubts functions ----------------*/

void funcL(){
   current_state_l = digitalRead(pinA_encoder_left);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_left) != current_state_l) { 
     count_left ++;
     encdir_l ="CW";
     
   } else {
     // Encoder is rotating counterclockwise
     count_left --;
     encdir_l ="CCW";
   }

}

void funcLift(){
   current_state_lift = digitalRead(pinA_encoder_lifting);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_lifting) != current_state_lift) { 
     count_lift ++;
     encdir_lift ="CW";
     
   } else {
     // Encoder is rotating counterclockwise
     count_lift --;
     encdir_lift ="CCW";
   }

}

void funcR(){
   current_state_r = digitalRead(pinA_encoder_right);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(pinB_encoder_right) != current_state_r) { 
     count_right ++;
     encdir_r ="CW";
     
   } else {
     // Encoder is rotating counterclockwise
     count_right --;
     encdir_r ="CCW";
   }

}

// Callback functions

void setSpeeds(const std_msgs::Int16MultiArray& speeds)
{
  short int *got = speeds.data;
  y = got[0];
  m = got[1];
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_speeds("speeds", &setSpeeds );

void backward(){
  analogWrite(pwm_left_motor,100);
  analogWrite(pwm_right_motor,100);
  digitalWrite(dir_left_motor, HIGH);
  digitalWrite(dir_right_motor, HIGH);
}

void forward(){
  analogWrite(pwm_left_motor,100);
  analogWrite(pwm_right_motor,100);
  digitalWrite(dir_left_motor, LOW);
  digitalWrite(dir_right_motor, LOW);
}

void right(){
  analogWrite(pwm_left_motor,70);
  analogWrite(pwm_right_motor,70);
  digitalWrite(dir_left_motor, LOW);
  digitalWrite(dir_right_motor, HIGH);
}

void left(){
  analogWrite(pwm_left_motor,70);
  analogWrite(pwm_right_motor,70);
  digitalWrite(dir_left_motor, HIGH);
  digitalWrite(dir_right_motor, LOW);
}
void stop_(){
  analogWrite(pwm_left_motor,0);
  analogWrite(pwm_right_motor,0);
  digitalWrite(dir_left_motor, HIGH);
  digitalWrite(dir_right_motor, LOW);
}


void setup() {
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
  pinMode(voltage_sensor, INPUT);
  pinMode(current_sensor, INPUT);

  //ISR for encoders declarations
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_right), funcR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_left), funcL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_lifting), funcLift,CHANGE);

  // Ros nodes declarations
  nh.initNode();
  nh.subscribe(sub_speeds);
  nh.advertise(pub_feedback);
  nh.advertise(pub_yaw);
  nh.advertise(pub_pitch);
  nh.advertise(pub_roll);
   
}

void loop() {
  // Calculating linear speeds for motors
  if ((millis()-prev_time) > speed_timer){
    speed_left = (count_left/450)*0.035;
    count_left = 0;

    speed_right = (count_right/450)*0.035;
    count_right = 0;
    
    prev_time = millis();
  }

  // calculating PWM for motors
  if ((millis() - last_action) > action_timer){
    pwm_left = pwm_left*epsi + (y-m)*(1 - epsi);
    pwm_right = pwm_right*epsi + (y+m)*(1 - epsi);
    
    if (pwm_left < 0){
      analogWrite(pwm_left_motor,abs(pwm_left));
      digitalWrite(dir_left_motor, HIGH);
    }
    else{
      analogWrite(pwm_left_motor,abs(pwm_left));
      digitalWrite(dir_left_motor, LOW);
    }
    
     if (pwm_right < 0){
      analogWrite(pwm_right_motor,abs(pwm_right));
      digitalWrite(dir_right_motor, HIGH);
    }
    else{
      analogWrite(pwm_right_motor,abs(pwm_right));
      digitalWrite(dir_right_motor, LOW);
    }
    last_action = millis();
    
  }

  // Sending IMU variables using ROS
  if ((millis() - prev_imu_timer) > imu_timer)
  {
    yaw.data = yaw_angle;
    pub_yaw.publish(&yaw);
    pitch.data = pitch_angle;
    pub_pitch.publish(&pitch);
    roll.data = roll_angle;
    pub_roll.publish(&roll);
    prev_imu_timer = millis();
  }

  // Sending feedback values of motors using ROS
  if ((millis() - prev_feedback_timer) > feedback_timer)
  {
    feedback.data_length = 2;
    feedback.data[0] = abs(pwm_left);
    feedback.data[1] = abs(pwm_right);
    
    pub_feedback.publish(&feedback);
  }
  
  nh.spinOnce();
}

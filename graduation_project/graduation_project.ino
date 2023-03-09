#define __STM32F1__
#include <ros.h>

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

// counts for  encoders
int count_left = 0, count_right = 0, count_lift = 0; 
double prev_time = 0, speed_timer = 1000;
double action_timer = 10, last_action = 0;

// Speeds for motors 
float speed_left = 0, speed_right = 0;

// IMU variables 
float yaw = 0, pitch = 0, roll = 0;

// States for encoders 
int current_state_l = 0, current_state_r = 0, current_state_lift = 0;

// direction of encoders 
String encdir_l ="", encdir_r ="", encdir_lift ="";  

// Speeds variables and damping factor
float y = 0, m = 0, epsi = 0.8;

// PWMs for motors
int pwm_left = 0, pwm_right = 0;

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
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_right), funcR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_left), funcL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_encoder_lifting), funcLift,CHANGE);
}

void loop() {
  if ((millis()-prev_time) > speed_timer){
    speed_left = (count_left/450)*0.035;
    count_left = 0;

    speed_right = (count_right/450)*0.035;
    count_right = 0;
    
    prev_time = millis();
  }
  
  if ((millis()-last_action) > action_timer){
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
    
  }
  
}

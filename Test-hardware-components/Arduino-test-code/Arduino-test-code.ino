#define motor1_dir PA0
#define motor2_dir PA2
#define motor1_pwm PA1
#define motor2_pwm PA3

#define motor1_encoder_int1 PA5
#define motor1_encoder_int2 PA6

int countA = 0, countB = 0;
int currentStateCLK;
int previousStateCLK; 

String encdir ="";
float last_time = 0, current_time = 0;
float RPM = 0;

void setup() {
  
  //  pinMode(motor1_dir, OUTPUT);
  //  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_encoder_int1,INPUT);
  pinMode(motor1_encoder_int2,INPUT);
  attachInterrupt(digitalPinToInterrupt(motor1_encoder_int1), fucnA,RISING);
  
  Serial.begin(9600);
}

void fucnA(){
  
   currentStateCLK = digitalRead(motor1_encoder_int1);
    
   // If the inputDT state is different than the inputCLK state then 
   // the encoder is rotating clockwise
   if (digitalRead(motor1_encoder_int2) != currentStateCLK) { 
     countA ++;
     encdir ="CW";
     
     
   } else {
     // Encoder is rotating counterclockwise
     countA --;
     encdir ="CCW";
     
   }

}

void loop() {

  //analogWrite(motor1_pwm, 255);
  //digitalWrite(motor1_dir,HIGH);
  current_time = millis();
  if ((current_time-last_time)>1000){
    RPM = (countA/230)*60;
    last_time = current_time;
    Serial.print("countA = ");
    Serial.println(countA);
    Serial.print("Dir: ");
    Serial.println(encdir);
    Serial.print("RPM: ");
    Serial.println(RPM);
    countA = 0;
    
  }

}

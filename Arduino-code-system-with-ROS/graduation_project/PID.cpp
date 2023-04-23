#include "PID.h"
PID::PID(bool isAngle)
{
    is_angle=isAngle;
};

void PID::set_constants(double kP, double kI, double kD)
{
  acc=0;
  last_time=0;
  kp = kP;
  ki = kI;
  kd = kD;
}

void PID::set_setpoint(double setPoint)
{
  setpoint = setPoint;
}

void PID::set_constrains(double lower_bound, double upper_bound)
{
  lower = lower_bound;
  upper = upper_bound;
}

double PID::calc(double mv)
{
  if(!enabled) return 0;
  double err = setpoint - mv;
  double dt = (millis() - last_time)/1000.0;

  if(is_angle)
  {
    while(err>180) err-=360;
    while(err<-180)err+=360;
  }

  double diff=0;
  if(last_time)
  {
      diff = (mv-last_mv);

      if(is_angle)
      {
        while(diff>180) diff-=360;
        while(diff<-180)diff+=360;
      }
      
      diff/=dt;
      acc+=err*dt;
      acc=constrain(acc,lower,upper);

  }
  last_time = millis();
  last_mv = mv;
 
  return (err*kp)+(acc*ki)-(diff*kd);

}

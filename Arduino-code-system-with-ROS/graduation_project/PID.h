#include <Arduino.h>
class PID
{
  public:
  PID(bool isAngle);
  double calc(double input);
  void set_constants(double kP, double kI, double kD);
  void set_setpoint(double setPoint);
  void set_constrains(double lower_bound, double upper_bound);
  private:
    bool is_angle = 0;
    double kp=0,ki=0,kd=0;

    double acc=0;

    double last_mv=0;
    long long last_time=0;

    double setpoint=0;
    double lower=-400, upper=400;

    bool enabled = 1;

};

> ## ***This is a documentation for issue #11***    
> `Testing hardware components using arduino ide`

## 1. Testing the 3 Motors using cytrons 1 channel.
- We provide speed and direction control of the 2 motors using 2 cytrons 1 channel, the third motor is used for the lifting mechanism so we only need direction control.

## 2. Getting encoders readings.
-   By comparing the two sets of pulses out of the encoder we can determine the direction of the motor.
-   Depending on the direction we count rising edges of the pulses by interrupt to evaluate revolution per minite. 
     - `RPM = ( counts / encoder resolution )*60`
- Finally, we display the readings of counts, RPM and Motor direction on a serial monitor every 1 sec.




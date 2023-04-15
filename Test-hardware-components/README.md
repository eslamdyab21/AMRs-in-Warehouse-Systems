> ## ***This is a documentation for issue #11***    
> `Testing hardware components using arduino ide`

## 1. Testing the 3 Motors using cytrons 1 channel.
- We provide speed and direction control of the 2 motors using 2 cytrons 1 channel, the third motor is used for the lifting mechanism so we only need direction control.

## 2. Getting encoders readings.
-   By comparing the two sets of pulses out of the encoder we can determine the direction of the motor.
-   Depending on the direction we count rising edges of the pulses by interrupt to evaluate revolution per minite. 
     - `RPM = ( counts / encoder resolution )*60`
- Finally, we display the readings of counts, RPM and Motor direction on a serial monitor every 1 sec.

## 3. Testing MPU6050 on the MicroController(Stm32).
-  MPU is used to rotate (90 or 180) degree (clockwise or Anticlockwise ) using the Yaw angle 
-  the sensor is tested 
    - we will make the connection like in the figure below then watch the reading , also by change the yaw angle of the sesnor the reading will change 

![BluePillMPU6050](https://user-images.githubusercontent.com/93758246/232246374-b5e06a53-7a0c-4528-87cf-b770c5909e74.png)


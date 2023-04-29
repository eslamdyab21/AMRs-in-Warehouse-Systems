> ### ***This is a documentation for issue #33***    
> `HAL Drivers`  
<br>

## Encoder driver
### We have 5 functions in this driver :
1. Motor direction
   ```C
   u8 HENCODER_u8GetMotorDirection(u8 Copy_u8Channel2Port , u8 Copy_u8Channel2Pin);
   ```
	- This function gets direction of the motor by comparing the two sets of pulses from the encoder *(state of the 2 pins)*.
	- It returns motor direction :
		- returning 1 means `ClockwiseDirection`
		- returning 2 means `CounterClockwiseDirection`
	- It takes 2 parameters :
		- Parameter1: port ID of encoder pin 
		- Parameter2: pin ID of encoder pin 

   <br>
2. Encoder counts 
   ```C
   void HENCODER_voidEncoderCounts(u8 Copy_u8Channel2Port ,u8 Copy_u8Channel2Pin);
   ```
	- This function counts trigger point `rising edge` depending on the motor direction
	- It takes 2 parameters :
		- Parameter1: port ID of encoder pin 
		- Parameter2: pin ID of encoder pin 
   <br>
3. Get encoder counts 
   ```C
   s32 HENCODER_s32GetEncoderCounts(void);
   ```
	- This function gets number of counts to be able to get RPM and distance moved
	- It returns number of counts depending on motor direction
   <br>
4. Get revolution per minite 
   ```C
   s32 HENCODER_s32GetRevPerMin(s32 Copy_s32EncoderCounts);
   ```
	- This function calculates and returns RPM value.  
		- RPM equation : *`RPM = (EncoderCounts*60)/EncoderResolution`*	
   <br>   
5. Get distance moved 
   ```C
   f32 HENCODER_f32GetDistance( s32 Copy_s32EncoderCounts );
   ```
	- This function calculates and returns distance moved. 
		- Distance equation : *`Distance = (EncoderCounts*2*Pi*WheelRadius)/EncoderResolution`*


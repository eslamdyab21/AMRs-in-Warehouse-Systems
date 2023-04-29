> ## ***This is a documentation for issue #35***    
> `MCAL internal peripherals`

## We have 8 drivers in this issue:
- Reset and clock controller (RCC)
- General purpose I/Os (GPIO)
- External interrupt (EXTI)
- Alternative function I/Os (AFIO)
- General purpose timers (TIM2 - TIM3)
- Analog to Digital converter (ADC)
- Direct memory access controller (DMA)
<br>

# 1. Reset and clock controller (RCC)
### This peripheral provides:
- Processor clock.
	- High speed internal (HSI)   :     RC    -    8MHz
	- High speed external (HSE) :     RC    -    25MHZ
	- High speed external (HSE) : crystal  - 4 to 16MHZ
	- Phase locked loop (PLL)     :       up to 72MHz
- Internal peripherals clock control.
<br>

## We have 3 functions in this driver
1. Initialization
   ```C
   void RCC_voidInitSysClock(void);
   ```
	- This function selects system clock.
		- Clock options : `RCC_HSE_CRYSTAL` , `RCC_HSE_RC` , `RCC_HSI` , `RCC_PLL`
		-  `RCC_PLL` options : `RCC_PLL_IN_HSI_DIV_2` , `RCC_PLL_IN_HSE_DIV_2` , `RCC_PLL_IN_HSE`

   <br>
2. Enable clock
   ```C
   void RCC_voidEnableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId);
   ```
	- This function enables selected clock on a specific peripheral
	- It takes 2 parameters:
		- Parameter1 : Bus used to connect peripheral to microcontroller
		- Parameter2 : index of peripheral
   <br>
3. Disable clock
   ```C
   void RCC_voidDisableClock(u8 Copy_u8BusId,u8 Copy_u8PeripheralId);
   ```
	- This function removes peripheral clock 
	- It takes 2 parameters:
		- Parameter1 : Bus used to connect peripheral to microcontroller
		- Parameter2 : index of peripheral
   <br>
# 2. General purpose I/Os (GPIO)
### This peripheral provides:
- 4 bits for each pin *(2 bits for mode , 2 bits for configuration)* 

	![GPIO](https://user-images.githubusercontent.com/68203785/235318256-28116ff6-b099-4409-97ed-8318d4e055ba.png)

- 4 modes for each pin 
	- input 
	- output - 2MHz
	- output - 10MHz
	- output - 50MHz
- 4 configurations depend on selected mode
	- if input mode
		- analog 
		- floating
		- pull up or pull down
	- if output mode
		- push pull
		- open drain 
		- alternative function push pull
		- alternative function open drain
 <br>
 
## We have 4 functions in this driver
1. Set pin direction
   ```C
   void MGPIO_VoidSetPinDirection(u8 Copy_u8Port, u8 Copy_u8Pin, u8 Copy_u8Mode);
   ```
	- This function specifies pin mode *(input or output)* and its configurations. 
	- It takes 3 Parameters:
		- Parameter1 : Port ID
		- Parameter2 : pin ID
		- Parameter3 : mode & configuration of pin

   <br>
2. Set pin value
   ```C
   void MGPIO_VoidSetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Value);
   ```
	- This function determines pin value *(high or low)*. 
	- It takes 3 parameters:
		- Parameter1 : Port ID
		- Parameter2 : pin ID
		- Parameter3 : pin value
   <br>
3. Get pin value
   ```C
   u8 MGPIO_u8GetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin);
   ```
	- This function gets the value of a specific pin. 
	- It returns the value of the pin and takes 2 parameters:
		- Parameter1 : Port ID
		- Parameter2 : pin ID
   <br>
4. lock pin
   ```C
   void MGPIO_VoidLockPin(u8 Copy_u8Port , u8 Copy_u8Pin);
   ```
	- This function locks the specified pin mode *(each lock bit of `LCKR` register freezes the corresponding 4 bits of the control registers `CRL` , `CRH`)* 
	- It takes 2 parameters:
		- Parameter1 : Port ID
		- Parameter2 : pin ID
   <br>
# 3. External interrupt (EXTI)
### This peripheral provides:
- 16 external interrupt/event lines
- Detection of external signal from external devices
- Generation of up to 20 software interrupt requests
- Dedicated status bit for each interrupt line 
<br>

## We have 6 functions in this driver
1. Initialization
   ```C
   void MEXTI_voidInit();
   ```
	- This function specifies the trigger point used to detect external signals
		- Trigger point options : rising edge , falling edge , on change

   <br>
2. Enable EXTI
   ```C
   void MEXTI_voidEnableEXTI(u8 Copy_u8EXTILine);
   ```
	- This function enables interrupt on a specific line. 
	- It takes 1 parameter: interrupt line ID
   <br>
3. Disable EXTI
   ```C
   void MEXTI_voidDisableEXTI(u8 Copy_u8EXTILine);
   ```
	- This function disables interrupt on a specific line. 
	- It takes 1 parameter: interrupt line ID
   <br>
4. Software trigger
   ```C
   void MEXTI_voidSoftwareTrigger(u8 Copy_u8EXTILine);
   ```
	- This function generates a software interrupt on a specific line. 
	- It takes 1 parameter: interrupt line ID
   <br>
5. Set signal latch
   ```C
   void MEXTI_voidSetSignalLatch(u8 Copy_u8EXTILine , u8 Copy_u8EXTIMode);
   ```
	- This function changes the interrupt mode *(trigger point)* and line in run time. 
	- It takes 2 parameter: 
		- Parameter1 : interrupt line ID
		- Parameter2 : trigger point of the interrupt
   <br>   
6. Call back function
   ```C
   void EXTI_voidSetCallBack(void (*ptr) (void) , u8 Copy_u8EXTILine);
   ```
	- This function changes the interrupt mode *(trigger point)* and line in run time. 
	- It takes 2 parameter: 
		- Parameter1 : pointer to ISR function 
		- Parameter2 : interrupt line ID
   <br>  
# 4. Alternative function I/Os (AFIO)
### This peripheral provides:
- Selection of a source input (PORT) for external interrupt line.

	![AFIO](https://user-images.githubusercontent.com/68203785/235318231-4120d814-3d89-496c-9416-132c2c3f1133.png)

<br>

## We have 1 function in this driver
1. interrupt configuration
   ```C
   void	MAFIO_voidSetEXTIConfiguration(u8 Copy_u8Line ,u8 Copy_u8PortMap);
   ```
	- This function specifies which port of a particular line is used as an interrupt. 
	- It takes 2 parameters:
		- Parameter1 : interrupt line ID
		- Parameter2 : port ID

   <br>
# 5. General purpose timers (TIM2 - TIM3)
### This peripheral provides:
- 4 independent channels for:  
	- Input capture
	- Output compare
	- PWM generation *(Edge and Center aligned modes)*
<br>

## We have 3 functions in TIM2 driver
1. Initialization
   ```C
   void MTIM2_voidInit (void);
   ```
	- This function: 
		- Enables PWM mode on channel 4 only 
		- Enables counter  
		- Loads prescaler value found in the config file

   <br>
2. Busy wait function
   ```C
   void MTIM2_voidSetBusyWait(u32 Copy_u16Ticks);
   ```
	- This function makes a time delay 
	- It takes 1 parameter: number of ticks 
   <br>
3. PWM function
   ```C
   void MTIM2_voidOutputPWM (u16 Copy_u16CompareValue);
   ```
	- This function provides speed control on a motor. 
	- It takes 1 parameter: duty cycle *(from 0 to 100)*
   <br>
### Note : 
- Both TIM2 and TIM3 have the same functions except we only implement *`PWM mode channel4`* in TIM2 and *`PWM mode channel1`* in TIM3 to control PWM pins in our PCB  
<br>

# 6. Analog to Digital converter (ADC)
### This peripheral provides:
- 12-bits resolution
- Single, continuous and discontinuous conversion modes
- Interrupt generation at end of Conversion and Analog watchdog event
- DMA request generation during regular channel conversion
<br>

## We have 2 functions in this driver
1. Initialization
   ```C
   void MADC1_voidInit(void);
   ```
	- This function: 
		- Specifies number of channels
		- Enables scan and DMA mode if dealing with more than one channel 
		- Generates interrupt at the end of each conversion

   <br>
2. conversion function
   ```C
   void MADC1_voidStartConversion(u8 Copy_u8ChannelID , u8 Copy_u8SeqID);
   ```
	- This function: 
		- Specifies the order of channels conversion
		- Loads the sampling time found in the config file
		- Enable ADC and self calibration 
	- It takes 2 parameters: 
		- Parameter1 : channel ID
		- Parameter2 : sequence ID 
   <br>
# 7. Direct memory access controller (DMA)
### This peripheral provides:
- 7 independently configurable channels.
- 3 event flags *(DMA Half Transfer, DMA Transfer complete and DMA Transfer Error)*.
- Independent source and destination transfer size.
- Circular mode to handle circular buffers and continuous data flows *(ADC scan mode)*.
<br>

## We have 2 functions in this driver
1. Initialization
   ```C
   void MDMA_voidChannel1Init(void);
   ```
	- This function defines channel configurations: 
		- Enables circular mode
		- Specifies peripheral & memory size and channel priority level
		- Enable memory increment mode

   <br>
2. conversion function
   ```C
   void MDMA_voidChannel1Start(u32 *Copy_Pointeru32SourceAddress, u32 *Copy_Pointer32DestinationAddress, u16 Copy_u16BlockLength);
   ```
	- This function copies data from source to destination
	- It takes 3 parameters: 
		- Parameter1 : pointer to source address of the peripheral data register from which the data will be read.
		- Parameter2 : pointer to destination address of the memory area to which the data will be written. 
		- Parameter3 : number of data to be transferred *(Block length)*
   <br>


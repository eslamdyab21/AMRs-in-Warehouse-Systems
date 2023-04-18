> ## ***This is a documentation for issue #35***    
> `MCAL internal peripherals`

## We have 6 drivers in this issue:
- RCC
- GPIO
- EXTI
- AFIO
- TIM2
- TIM3
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
	- Description : Select system clock.
		- Clock options : `RCC_HSE_CRYSTAL` , `RCC_HSE_RC` , `RCC_HSI` , `RCC_PLL`
		-  `RCC_PLL` options : `RCC_PLL_IN_HSI_DIV_2` , `RCC_PLL_IN_HSE_DIV_2` , `RCC_PLL_IN_HSE`

   <br>
2. Enable clock
   ```C
   void RCC_voidEnableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId);
   ```
	- Parameter : Bus used to connect peripheral to microcontroller , index of peripheral
	- Description : Enable selected clock on a specific peripheral.
   <br>
3. Disable clock
   ```C
   void RCC_voidDisableClock(u8 Copy_u8BusId,u8 Copy_u8PeripheralId);
   ```
	- Parameter : Bus used to connect peripheral to microcontroller , index of peripheral
	- Description : remove peripheral clock.
   <br>
# 2. General purpose I/Os (GPIO)
### This peripheral provides:
- 4 bits for each pin (2 bits for mode , 2 bits for configuration) 

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
	- Parameters : Port ID , pin ID , mode & configuration of pin.
	- Description : specify pin mode (input or output) and its configurations.

   <br>
2. Set pin value
   ```C
   void MGPIO_VoidSetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Value);
   ```
	- Parameter : Port ID , pin ID , pin value.
	- Description : determine pin value (high or low).
   <br>
3. Get pin value
   ```C
   u8 MGPIO_u8GetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin);
   ```
	- Parameter : Port ID , pin ID.
	- Return : value of pin
	- Description : Get the value of a specific pin.
   <br>
4. lock pin
   ```C
   void MGPIO_VoidLockPin(u8 Copy_u8Port , u8 Copy_u8Pin);
   ```
	- Parameter : Port ID , pin ID.
	- Description : Lock pin mode - each lock bit freezes the corresponding 4 bits of the control registers `CRL` , `CRH`.
   <br>

> ## ***This is a documentation for issue #12***    
> `MCAL core peripherals`

## We have 2 core drivers in this issue:
- NVIC
- SYSTICK
<br>

# 1. Nested vectored interrupt controller (NVIC)
### This peripheral provides:
- Reduction of interrupt latency.
- Normal nesting interrupt support.
- Independent enable pin and software set flag for each external interrupt.
- software and hardware priority support.
<br>

## We have 6 functions in this driver
1. Enable interrupt.
   ```C
   void MNVIC_voidEnableInterrupt  (u8 Copy_u8IntNumber);
   ```  
	- Parameter : Index of interrupt (position).
	- Description : Set NVIC enable pin for the interrupt.

   <br>
2. Disable interrupt.
   ```C
   void MNVIC_voidDisableInterrupt (u8 Copy_u8IntNumber);
   ```  
	- Parameter : Index of interrupt (position).
	- Description : Disable the interrupt.
   <br>
3. Set pending flag.
   ```C
   void MNVIC_voidSetPendingFlag   (u8 Copy_u8IntNumber);
   ```  
	- Parameter : Index of interrupt (position).
	- Description : Set interrupt pending flag by software for debugging and testing.
   <br>
4. Clear pending flag.
   ```C
   void MNVIC_voidClearPendingFlag (u8 Copy_u8IntNumber);
   ```  
	- Parameter : index of interrupt (position).
	- Description : Clear interrupt pending flag.
   <br>
5. Active status.
   ```C
   u8 MNVIC_u8GetActiveFlag        (u8 Copy_u8IntNumber);
   ```  
	- Parameter : index of interrupt (position).
	- Return : status of active flag of the interrupt
		- 0 : inactive
		- 1 : active
	- Description : Get status of active flag to see if the interrupt is excuting or not.
   <br>
6. Priority level.
   ```C
   void MNVIC_voidSetPriority(u8 Copy_u8IntNumber , u8 Copy_u8GroupPriority ,u8 Copy_u8SubPriority );
   ```
	- Parameters : index of interrupt (position), number of bits for group priority, number of bits for sub priority.
	- Description : Specify group and sub priority level of each interrupt.
	- Note : SCB core peripheral defines 4 bits of priority classifications.  
<br>
<br>

# 2. System Tick (SYSTICK)
### This peripheral provides:
- Timer only mode (count down only).
- 2 clocks: `AHB` or `AHB\8` depending on the processor clock.
<br>

## We have 7 functions in this driver
1. Initialization function 
   ```C
   void MSTK_voidInit(void);
   ```  
	- Description : Select suitable clock - `AHB` or `AHB\8`.

   <br>
2. Busy wait function
   ```C
   void MSTK_voidSetBusyWait( u32 Copy_u32Ticks );
   ```  
	- Parameter : number of ticks to be counted.
	- Description : make a time delay.
	- Note : It is considered as a synchronous function as it lock the processor for N counts.
   <br>
3. Interval single function
   ```C
   void MSTK_voidSetIntervalSingle  ( u32 Copy_u32Ticks, void (*Copy_ptr)(void) );
   ```  
	- Parameter : Number of ticks to be counted down, pointer to function to be executed after timeout.
	- Description : Count once then after timeout, it jumps to ISR and execute it.
   <br>
4. Interval periodic function
   ```C
   void MSTK_voidSetIntervalPeriodic( u32 Copy_u32Ticks, void (*Copy_ptr)(void) );
   ```  
	- Parameter : Number of ticks to be counted, pointer to function to be executed after timeout.
	- Description : Count and execute the ISR periodically after a certain period of time.
   <br>
5. Stop interval function
   ```C
   void MSTK_voidStopInterval(void);
   ```  
	- Description : Disable SYSTICK interrupt and stop the timer.
   <br>
6. Get elapsed function
   ```C
   u32  MSTK_u32GetElapsedTime(void);
   ```  
	- Return : counted time.
	- Description : Get the counted time (load register - value register).
   <br>
7. Get remaining time function
   ```C
   u32  MSTK_u32GetRemainingTime(void);
   ```  
	- Return : remaining time.
	- Description : Get the remaining time (value register).


<br>

## Note: 
### If we want a delay of 1 sec then:
- RCC clock type should be `RCC_HSE_CRYSTAL`
- Control register of SYSTICK should have a clock of `AHB/8`  
`This makes 1 Tick (count) = 1 usec so 1 sec = 1000000 Ticks`


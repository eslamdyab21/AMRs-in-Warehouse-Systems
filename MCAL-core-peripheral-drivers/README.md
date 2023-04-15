> ## ***This is a documentation for issue #12***    
> `MCAL core peripherals`
<br>

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
2. Disable interrupt.
3. Set pending flag.
	- Set the interrupt flag by software.
	- Used for debugging and testing.
4. Clear pending flag.
5. Active status.
	- To see if the interrupt is excuting or not.
6. Priority level.
	- Group priority and sub priority.
	- SCB core peripheral defines 4 bits of priority classifications.  
<br>
<br>

# 2. System Tick (SYSTICK)
### This peripheral provides:
- Timer only mode (count down only).
- 2 clocks: `AHB` or `AHB\8` depending on the processor clock.
<br>

## We have 7 functions in this driver
1. Initialization function 
	- To choose the suitable clock.
2. Busy wait function
	- it makes a time delay.
	- It is considered as a synchronous function as it lock the processor for N counts.
3. Interval single function
	-  count once then after the time is up, it jumps to ISR and execute it.
4. Interval periodic function
	- count and execute the ISR periodically after a certain period of time.
5. Stop interval function
	- To disable SYSTICK interrupt and stop the timer.
6. Get elapsed function
	- To get the counted time (load register - value register).
7. Get remaining time function
	- To get the remaining time (value register).
<br>

## Note: 
### If we want a delay of 1 sec then:
- RCC clock type should be `RCC_HSE_CRYSTAL`
- Control register of SYSTICK should have a clock of `AHB/8`  
`This makes 1 Tick (count) = 1 usec so 1 sec = 1000000 Ticks`


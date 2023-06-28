################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/UART2/USART_program.c 

OBJS += \
./Src/UART2/USART_program.o 

C_DEPS += \
./Src/UART2/USART_program.d 


# Each subdirectory must supply rules for building sources it contributes
Src/UART2/%.o Src/UART2/%.su Src/UART2/%.cyclo: ../Src/UART2/%.c Src/UART2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-UART2

clean-Src-2f-UART2:
	-$(RM) ./Src/UART2/USART_program.cyclo ./Src/UART2/USART_program.d ./Src/UART2/USART_program.o ./Src/UART2/USART_program.su

.PHONY: clean-Src-2f-UART2


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/TIM2/TIM2_program.c 

OBJS += \
./Src/TIM2/TIM2_program.o 

C_DEPS += \
./Src/TIM2/TIM2_program.d 


# Each subdirectory must supply rules for building sources it contributes
Src/TIM2/%.o Src/TIM2/%.su Src/TIM2/%.cyclo: ../Src/TIM2/%.c Src/TIM2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-TIM2

clean-Src-2f-TIM2:
	-$(RM) ./Src/TIM2/TIM2_program.cyclo ./Src/TIM2/TIM2_program.d ./Src/TIM2/TIM2_program.o ./Src/TIM2/TIM2_program.su

.PHONY: clean-Src-2f-TIM2


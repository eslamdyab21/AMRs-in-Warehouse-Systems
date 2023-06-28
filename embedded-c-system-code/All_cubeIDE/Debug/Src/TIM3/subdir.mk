################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/TIM3/TIM3_program.c 

OBJS += \
./Src/TIM3/TIM3_program.o 

C_DEPS += \
./Src/TIM3/TIM3_program.d 


# Each subdirectory must supply rules for building sources it contributes
Src/TIM3/%.o Src/TIM3/%.su Src/TIM3/%.cyclo: ../Src/TIM3/%.c Src/TIM3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-TIM3

clean-Src-2f-TIM3:
	-$(RM) ./Src/TIM3/TIM3_program.cyclo ./Src/TIM3/TIM3_program.d ./Src/TIM3/TIM3_program.o ./Src/TIM3/TIM3_program.su

.PHONY: clean-Src-2f-TIM3


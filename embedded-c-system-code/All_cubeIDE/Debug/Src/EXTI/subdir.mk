################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/EXTI/EXTI_program.c 

OBJS += \
./Src/EXTI/EXTI_program.o 

C_DEPS += \
./Src/EXTI/EXTI_program.d 


# Each subdirectory must supply rules for building sources it contributes
Src/EXTI/%.o Src/EXTI/%.su Src/EXTI/%.cyclo: ../Src/EXTI/%.c Src/EXTI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-EXTI

clean-Src-2f-EXTI:
	-$(RM) ./Src/EXTI/EXTI_program.cyclo ./Src/EXTI/EXTI_program.d ./Src/EXTI/EXTI_program.o ./Src/EXTI/EXTI_program.su

.PHONY: clean-Src-2f-EXTI


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/NVIC/NVIC_program.c 

OBJS += \
./Src/NVIC/NVIC_program.o 

C_DEPS += \
./Src/NVIC/NVIC_program.d 


# Each subdirectory must supply rules for building sources it contributes
Src/NVIC/%.o Src/NVIC/%.su Src/NVIC/%.cyclo: ../Src/NVIC/%.c Src/NVIC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-NVIC

clean-Src-2f-NVIC:
	-$(RM) ./Src/NVIC/NVIC_program.cyclo ./Src/NVIC/NVIC_program.d ./Src/NVIC/NVIC_program.o ./Src/NVIC/NVIC_program.su

.PHONY: clean-Src-2f-NVIC


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RNS_interface/RNS_interface.c 

OBJS += \
./Core/Src/RNS_interface/RNS_interface.o 

C_DEPS += \
./Core/Src/RNS_interface/RNS_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/RNS_interface/%.o: ../Core/Src/RNS_interface/%.c Core/Src/RNS_interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-RNS_interface

clean-Core-2f-Src-2f-RNS_interface:
	-$(RM) ./Core/Src/RNS_interface/RNS_interface.d ./Core/Src/RNS_interface/RNS_interface.o

.PHONY: clean-Core-2f-Src-2f-RNS_interface


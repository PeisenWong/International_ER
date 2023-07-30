################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DTLOG/dtlog.c 

OBJS += \
./Core/Src/DTLOG/dtlog.o 

C_DEPS += \
./Core/Src/DTLOG/dtlog.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DTLOG/%.o: ../Core/Src/DTLOG/%.c Core/Src/DTLOG/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DTLOG

clean-Core-2f-Src-2f-DTLOG:
	-$(RM) ./Core/Src/DTLOG/dtlog.d ./Core/Src/DTLOG/dtlog.o

.PHONY: clean-Core-2f-Src-2f-DTLOG


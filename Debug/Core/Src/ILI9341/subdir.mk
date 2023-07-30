################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ILI9341/ILI9341_Driver.c 

OBJS += \
./Core/Src/ILI9341/ILI9341_Driver.o 

C_DEPS += \
./Core/Src/ILI9341/ILI9341_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ILI9341/%.o: ../Core/Src/ILI9341/%.c Core/Src/ILI9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ILI9341

clean-Core-2f-Src-2f-ILI9341:
	-$(RM) ./Core/Src/ILI9341/ILI9341_Driver.d ./Core/Src/ILI9341/ILI9341_Driver.o

.PHONY: clean-Core-2f-Src-2f-ILI9341


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VESC_CAN/bldc_interface.c \
../Core/Src/VESC_CAN/buffer.c \
../Core/Src/VESC_CAN/crc.c \
../Core/Src/VESC_CAN/vesc_can.c \
../Core/Src/VESC_CAN/vesc_interface.c 

OBJS += \
./Core/Src/VESC_CAN/bldc_interface.o \
./Core/Src/VESC_CAN/buffer.o \
./Core/Src/VESC_CAN/crc.o \
./Core/Src/VESC_CAN/vesc_can.o \
./Core/Src/VESC_CAN/vesc_interface.o 

C_DEPS += \
./Core/Src/VESC_CAN/bldc_interface.d \
./Core/Src/VESC_CAN/buffer.d \
./Core/Src/VESC_CAN/crc.d \
./Core/Src/VESC_CAN/vesc_can.d \
./Core/Src/VESC_CAN/vesc_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/VESC_CAN/%.o: ../Core/Src/VESC_CAN/%.c Core/Src/VESC_CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-VESC_CAN

clean-Core-2f-Src-2f-VESC_CAN:
	-$(RM) ./Core/Src/VESC_CAN/bldc_interface.d ./Core/Src/VESC_CAN/bldc_interface.o ./Core/Src/VESC_CAN/buffer.d ./Core/Src/VESC_CAN/buffer.o ./Core/Src/VESC_CAN/crc.d ./Core/Src/VESC_CAN/crc.o ./Core/Src/VESC_CAN/vesc_can.d ./Core/Src/VESC_CAN/vesc_can.o ./Core/Src/VESC_CAN/vesc_interface.d ./Core/Src/VESC_CAN/vesc_interface.o

.PHONY: clean-Core-2f-Src-2f-VESC_CAN


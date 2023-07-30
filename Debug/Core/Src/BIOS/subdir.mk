################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BIOS/bdc.c \
../Core/Src/BIOS/gpio.c \
../Core/Src/BIOS/multiplexer.c \
../Core/Src/BIOS/pwm.c \
../Core/Src/BIOS/qei.c \
../Core/Src/BIOS/shiftreg.c \
../Core/Src/BIOS/system.c \
../Core/Src/BIOS/timer.c \
../Core/Src/BIOS/uart.c 

OBJS += \
./Core/Src/BIOS/bdc.o \
./Core/Src/BIOS/gpio.o \
./Core/Src/BIOS/multiplexer.o \
./Core/Src/BIOS/pwm.o \
./Core/Src/BIOS/qei.o \
./Core/Src/BIOS/shiftreg.o \
./Core/Src/BIOS/system.o \
./Core/Src/BIOS/timer.o \
./Core/Src/BIOS/uart.o 

C_DEPS += \
./Core/Src/BIOS/bdc.d \
./Core/Src/BIOS/gpio.d \
./Core/Src/BIOS/multiplexer.d \
./Core/Src/BIOS/pwm.d \
./Core/Src/BIOS/qei.d \
./Core/Src/BIOS/shiftreg.d \
./Core/Src/BIOS/system.d \
./Core/Src/BIOS/timer.d \
./Core/Src/BIOS/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BIOS/%.o: ../Core/Src/BIOS/%.c Core/Src/BIOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BIOS

clean-Core-2f-Src-2f-BIOS:
	-$(RM) ./Core/Src/BIOS/bdc.d ./Core/Src/BIOS/bdc.o ./Core/Src/BIOS/gpio.d ./Core/Src/BIOS/gpio.o ./Core/Src/BIOS/multiplexer.d ./Core/Src/BIOS/multiplexer.o ./Core/Src/BIOS/pwm.d ./Core/Src/BIOS/pwm.o ./Core/Src/BIOS/qei.d ./Core/Src/BIOS/qei.o ./Core/Src/BIOS/shiftreg.d ./Core/Src/BIOS/shiftreg.o ./Core/Src/BIOS/system.d ./Core/Src/BIOS/system.o ./Core/Src/BIOS/timer.d ./Core/Src/BIOS/timer.o ./Core/Src/BIOS/uart.d ./Core/Src/BIOS/uart.o

.PHONY: clean-Core-2f-Src-2f-BIOS


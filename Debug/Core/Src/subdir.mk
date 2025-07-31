################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bme280.c \
../Core/Src/bmi088.c \
../Core/Src/flight_algorithm.c \
../Core/Src/kalman.c \
../Core/Src/lora.c \
../Core/Src/main.c \
../Core/Src/packet.c \
../Core/Src/quaternion.c \
../Core/Src/queternion.c \
../Core/Src/sensor_fusion.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test_modes.c \
../Core/Src/uart_handler.c 

OBJS += \
./Core/Src/bme280.o \
./Core/Src/bmi088.o \
./Core/Src/flight_algorithm.o \
./Core/Src/kalman.o \
./Core/Src/lora.o \
./Core/Src/main.o \
./Core/Src/packet.o \
./Core/Src/quaternion.o \
./Core/Src/queternion.o \
./Core/Src/sensor_fusion.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test_modes.o \
./Core/Src/uart_handler.o 

C_DEPS += \
./Core/Src/bme280.d \
./Core/Src/bmi088.d \
./Core/Src/flight_algorithm.d \
./Core/Src/kalman.d \
./Core/Src/lora.d \
./Core/Src/main.d \
./Core/Src/packet.d \
./Core/Src/quaternion.d \
./Core/Src/queternion.d \
./Core/Src/sensor_fusion.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test_modes.d \
./Core/Src/uart_handler.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bme280.cyclo ./Core/Src/bme280.d ./Core/Src/bme280.o ./Core/Src/bme280.su ./Core/Src/bmi088.cyclo ./Core/Src/bmi088.d ./Core/Src/bmi088.o ./Core/Src/bmi088.su ./Core/Src/flight_algorithm.cyclo ./Core/Src/flight_algorithm.d ./Core/Src/flight_algorithm.o ./Core/Src/flight_algorithm.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/lora.cyclo ./Core/Src/lora.d ./Core/Src/lora.o ./Core/Src/lora.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/packet.cyclo ./Core/Src/packet.d ./Core/Src/packet.o ./Core/Src/packet.su ./Core/Src/quaternion.cyclo ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/queternion.cyclo ./Core/Src/queternion.d ./Core/Src/queternion.o ./Core/Src/queternion.su ./Core/Src/sensor_fusion.cyclo ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/test_modes.cyclo ./Core/Src/test_modes.d ./Core/Src/test_modes.o ./Core/Src/test_modes.su ./Core/Src/uart_handler.cyclo ./Core/Src/uart_handler.d ./Core/Src/uart_handler.o ./Core/Src/uart_handler.su

.PHONY: clean-Core-2f-Src


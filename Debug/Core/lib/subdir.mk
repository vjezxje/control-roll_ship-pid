################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/bno055.c \
../Core/lib/config_data.c \
../Core/lib/esc.c \
../Core/lib/nrf24l01p.c 

OBJS += \
./Core/lib/bno055.o \
./Core/lib/config_data.o \
./Core/lib/esc.o \
./Core/lib/nrf24l01p.o 

C_DEPS += \
./Core/lib/bno055.d \
./Core/lib/config_data.d \
./Core/lib/esc.d \
./Core/lib/nrf24l01p.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o Core/lib/%.su Core/lib/%.cyclo: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/cubeIDE/Board_V1/Core/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/bno055.cyclo ./Core/lib/bno055.d ./Core/lib/bno055.o ./Core/lib/bno055.su ./Core/lib/config_data.cyclo ./Core/lib/config_data.d ./Core/lib/config_data.o ./Core/lib/config_data.su ./Core/lib/esc.cyclo ./Core/lib/esc.d ./Core/lib/esc.o ./Core/lib/esc.su ./Core/lib/nrf24l01p.cyclo ./Core/lib/nrf24l01p.d ./Core/lib/nrf24l01p.o ./Core/lib/nrf24l01p.su

.PHONY: clean-Core-2f-lib


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_DEPS += \
./VL53L0X/platform/src/vl53l0x_platform.d \
./VL53L0X/platform/src/vl53l0x_platform_log.d 

OBJS += \
./VL53L0X/platform/src/vl53l0x_platform.o \
./VL53L0X/platform/src/vl53l0x_platform_log.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X/platform/src/vl53l0x_platform.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/platform/src/vl53l0x_platform.c VL53L0X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X/platform/src/vl53l0x_platform_log.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/platform/src/vl53l0x_platform_log.c VL53L0X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X-2f-platform-2f-src

clean-VL53L0X-2f-platform-2f-src:
	-$(RM) ./VL53L0X/platform/src/vl53l0x_platform.d ./VL53L0X/platform/src/vl53l0x_platform.o ./VL53L0X/platform/src/vl53l0x_platform.su ./VL53L0X/platform/src/vl53l0x_platform_log.d ./VL53L0X/platform/src/vl53l0x_platform_log.o ./VL53L0X/platform/src/vl53l0x_platform_log.su

.PHONY: clean-VL53L0X-2f-platform-2f-src


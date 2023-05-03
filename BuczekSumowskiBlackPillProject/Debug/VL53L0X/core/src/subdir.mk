################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_DEPS += \
./VL53L0X/core/src/vl53l0x_api.d \
./VL53L0X/core/src/vl53l0x_api_calibration.d \
./VL53L0X/core/src/vl53l0x_api_core.d \
./VL53L0X/core/src/vl53l0x_api_ranging.d \
./VL53L0X/core/src/vl53l0x_api_strings.d 

OBJS += \
./VL53L0X/core/src/vl53l0x_api.o \
./VL53L0X/core/src/vl53l0x_api_calibration.o \
./VL53L0X/core/src/vl53l0x_api_core.o \
./VL53L0X/core/src/vl53l0x_api_ranging.o \
./VL53L0X/core/src/vl53l0x_api_strings.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X/core/src/vl53l0x_api.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/core/src/vl53l0x_api.c VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X/core/src/vl53l0x_api_calibration.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/core/src/vl53l0x_api_calibration.c VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X/core/src/vl53l0x_api_core.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/core/src/vl53l0x_api_core.c VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X/core/src/vl53l0x_api_ranging.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/core/src/vl53l0x_api_ranging.c VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X/core/src/vl53l0x_api_strings.o: /home/pavel/Documents/SchoolRobots/BuczekSumowski/BuczekSumowskiSTM32Project/Drivers/VL53L0X/core/src/vl53l0x_api_strings.c VL53L0X/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0X/platform/inc -I../Drivers/VL53L0X/core/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X-2f-core-2f-src

clean-VL53L0X-2f-core-2f-src:
	-$(RM) ./VL53L0X/core/src/vl53l0x_api.d ./VL53L0X/core/src/vl53l0x_api.o ./VL53L0X/core/src/vl53l0x_api.su ./VL53L0X/core/src/vl53l0x_api_calibration.d ./VL53L0X/core/src/vl53l0x_api_calibration.o ./VL53L0X/core/src/vl53l0x_api_calibration.su ./VL53L0X/core/src/vl53l0x_api_core.d ./VL53L0X/core/src/vl53l0x_api_core.o ./VL53L0X/core/src/vl53l0x_api_core.su ./VL53L0X/core/src/vl53l0x_api_ranging.d ./VL53L0X/core/src/vl53l0x_api_ranging.o ./VL53L0X/core/src/vl53l0x_api_ranging.su ./VL53L0X/core/src/vl53l0x_api_strings.d ./VL53L0X/core/src/vl53l0x_api_strings.o ./VL53L0X/core/src/vl53l0x_api_strings.su

.PHONY: clean-VL53L0X-2f-core-2f-src


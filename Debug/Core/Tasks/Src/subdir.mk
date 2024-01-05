################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/Src/TaskBluetooth.c \
../Core/Tasks/Src/TaskBuzzer.c \
../Core/Tasks/Src/TaskEeprom.c \
../Core/Tasks/Src/TaskTemperatura.c \
../Core/Tasks/Src/TaskTimer.c 

OBJS += \
./Core/Tasks/Src/TaskBluetooth.o \
./Core/Tasks/Src/TaskBuzzer.o \
./Core/Tasks/Src/TaskEeprom.o \
./Core/Tasks/Src/TaskTemperatura.o \
./Core/Tasks/Src/TaskTimer.o 

C_DEPS += \
./Core/Tasks/Src/TaskBluetooth.d \
./Core/Tasks/Src/TaskBuzzer.d \
./Core/Tasks/Src/TaskEeprom.d \
./Core/Tasks/Src/TaskTemperatura.d \
./Core/Tasks/Src/TaskTimer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/Src/%.o Core/Tasks/Src/%.su Core/Tasks/Src/%.cyclo: ../Core/Tasks/Src/%.c Core/Tasks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/lucas/STM32CubeIDE/workspace_1.12.1/EasyPizza_v2/Core/Tasks/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Tasks-2f-Src

clean-Core-2f-Tasks-2f-Src:
	-$(RM) ./Core/Tasks/Src/TaskBluetooth.cyclo ./Core/Tasks/Src/TaskBluetooth.d ./Core/Tasks/Src/TaskBluetooth.o ./Core/Tasks/Src/TaskBluetooth.su ./Core/Tasks/Src/TaskBuzzer.cyclo ./Core/Tasks/Src/TaskBuzzer.d ./Core/Tasks/Src/TaskBuzzer.o ./Core/Tasks/Src/TaskBuzzer.su ./Core/Tasks/Src/TaskEeprom.cyclo ./Core/Tasks/Src/TaskEeprom.d ./Core/Tasks/Src/TaskEeprom.o ./Core/Tasks/Src/TaskEeprom.su ./Core/Tasks/Src/TaskTemperatura.cyclo ./Core/Tasks/Src/TaskTemperatura.d ./Core/Tasks/Src/TaskTemperatura.o ./Core/Tasks/Src/TaskTemperatura.su ./Core/Tasks/Src/TaskTimer.cyclo ./Core/Tasks/Src/TaskTimer.d ./Core/Tasks/Src/TaskTimer.o ./Core/Tasks/Src/TaskTimer.su

.PHONY: clean-Core-2f-Tasks-2f-Src


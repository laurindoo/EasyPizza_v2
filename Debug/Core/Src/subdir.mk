################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Bluetooth.c \
../Core/Src/Conversoes.c \
../Core/Src/Crc.c \
../Core/Src/Eeprom.c \
../Core/Src/OutputDigital.c \
../Core/Src/dwin.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_hal_timebase_tim.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/Bluetooth.o \
./Core/Src/Conversoes.o \
./Core/Src/Crc.o \
./Core/Src/Eeprom.o \
./Core/Src/OutputDigital.o \
./Core/Src/dwin.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_hal_timebase_tim.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/Bluetooth.d \
./Core/Src/Conversoes.d \
./Core/Src/Crc.d \
./Core/Src/Eeprom.d \
./Core/Src/OutputDigital.d \
./Core/Src/dwin.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_hal_timebase_tim.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/lucas/STM32CubeIDE/workspace_1.12.1/EasyPizza_v2/Core/Tasks/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Bluetooth.cyclo ./Core/Src/Bluetooth.d ./Core/Src/Bluetooth.o ./Core/Src/Bluetooth.su ./Core/Src/Conversoes.cyclo ./Core/Src/Conversoes.d ./Core/Src/Conversoes.o ./Core/Src/Conversoes.su ./Core/Src/Crc.cyclo ./Core/Src/Crc.d ./Core/Src/Crc.o ./Core/Src/Crc.su ./Core/Src/Eeprom.cyclo ./Core/Src/Eeprom.d ./Core/Src/Eeprom.o ./Core/Src/Eeprom.su ./Core/Src/OutputDigital.cyclo ./Core/Src/OutputDigital.d ./Core/Src/OutputDigital.o ./Core/Src/OutputDigital.su ./Core/Src/dwin.cyclo ./Core/Src/dwin.d ./Core/Src/dwin.o ./Core/Src/dwin.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_hal_timebase_tim.cyclo ./Core/Src/stm32f1xx_hal_timebase_tim.d ./Core/Src/stm32f1xx_hal_timebase_tim.o ./Core/Src/stm32f1xx_hal_timebase_tim.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/can.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/can.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/can.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/adc.o: ../Core/Src/adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/adc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/can.o: ../Core/Src/can.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/can.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/gpio.o: ../Core/Src/gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_hal_msp.o: ../Core/Src/stm32f4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_it.o: ../Core/Src/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32f4xx.o: ../Core/Src/system_stm32f4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/tim.o: ../Core/Src/tim.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/tim.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/usart.o: ../Core/Src/usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/usart.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/lib_unit_conversion/src/lib_unit_conversion.c 

OBJS += \
./lib/lib_unit_conversion/src/lib_unit_conversion.o 

C_DEPS += \
./lib/lib_unit_conversion/src/lib_unit_conversion.d 


# Each subdirectory must supply rules for building sources it contributes
lib/lib_unit_conversion/src/lib_unit_conversion.o: ../lib/lib_unit_conversion/src/lib_unit_conversion.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DFORD_FOCUS_STRS_2013_2018 -DUSE_LIB_CAN_BUS_SNIFFER -DUSE_HAL_DRIVER -DLIMIT_PIDS -DUSE_LIB_OBDII -DSTM32F446xx -DDEBUG -DSPOOF_DATA -c -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_unit_conversion/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_obdii/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_digital_dash/inc" -I../Core/Inc -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_ke_protocol/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_pid/inc" -I"C:/Users/Matth/Documents/git/GitHub/digital-dash-firmware/lib/lib_can_bus_sniffer/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/lib_unit_conversion/src/lib_unit_conversion.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


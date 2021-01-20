################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include lib/lib_unit_conversion/src/subdir.mk
-include lib/lib_pid/src/subdir.mk
-include lib/lib_obdii/src/subdir.mk
-include lib/lib_ke_protocol/src/subdir.mk
-include lib/lib_digital_dash/src/subdir.mk
-include lib/lib_can_bus_sniffer/src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
digital-dash-firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
digital-dash-firmware.list \

OBJCOPY_HEX += \
digital-dash-firmware.hex \

OBJCOPY_BIN += \
digital-dash-firmware.bin \


# All Target
all: digital-dash-firmware.elf secondary-outputs

# Tool invocations
digital-dash-firmware.elf: $(OBJS) $(USER_OBJS) C:\Users\Matth\Documents\git\GitHub\digital-dash-firmware\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "digital-dash-firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Matth\Documents\git\GitHub\digital-dash-firmware\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="digital-dash-firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

digital-dash-firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "digital-dash-firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

digital-dash-firmware.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "digital-dash-firmware.hex"
	@echo 'Finished building: $@'
	@echo ' '

digital-dash-firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "digital-dash-firmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

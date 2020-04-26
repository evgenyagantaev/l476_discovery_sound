################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
l476_discovery_sound_00_01.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
l476_discovery_sound_00_01.list \

OBJCOPY_BIN += \
l476_discovery_sound_00_01.bin \


# All Target
all: l476_discovery_sound_00_01.elf secondary-outputs

# Tool invocations
l476_discovery_sound_00_01.elf: $(OBJS) $(USER_OBJS) D:\workspace\stm32\cubeide\workspace\l476_discovery_sound_00_01\STM32L476VGTX_FLASH.ld
	arm-none-eabi-gcc -o "l476_discovery_sound_00_01.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\workspace\stm32\cubeide\workspace\l476_discovery_sound_00_01\STM32L476VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="l476_discovery_sound_00_01.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

l476_discovery_sound_00_01.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "l476_discovery_sound_00_01.list"
	@echo 'Finished building: $@'
	@echo ' '

l476_discovery_sound_00_01.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "l476_discovery_sound_00_01.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

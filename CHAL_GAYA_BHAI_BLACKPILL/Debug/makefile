################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := CHAL_GAYA_BHAI_BLACKPILL
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CHAL_GAYA_BHAI_BLACKPILL.elf \

MAP_FILES += \
CHAL_GAYA_BHAI_BLACKPILL.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CHAL_GAYA_BHAI_BLACKPILL.list \

OBJCOPY_HEX += \
CHAL_GAYA_BHAI_BLACKPILL.hex \

OBJCOPY_BIN += \
CHAL_GAYA_BHAI_BLACKPILL.bin \


# All Target
all: main-build

# Main-build Target
main-build: CHAL_GAYA_BHAI_BLACKPILL.elf secondary-outputs

# Tool invocations
CHAL_GAYA_BHAI_BLACKPILL.elf CHAL_GAYA_BHAI_BLACKPILL.map: $(OBJS) $(USER_OBJS) C:\Users\HP\STM32CubeIDE\workspace_1.12.1\CHAL_GAYA_BHAI_BLACKPILL\STM32F411CEUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "CHAL_GAYA_BHAI_BLACKPILL.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\HP\STM32CubeIDE\workspace_1.12.1\CHAL_GAYA_BHAI_BLACKPILL\STM32F411CEUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CHAL_GAYA_BHAI_BLACKPILL.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CHAL_GAYA_BHAI_BLACKPILL.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CHAL_GAYA_BHAI_BLACKPILL.list"
	@echo 'Finished building: $@'
	@echo ' '

CHAL_GAYA_BHAI_BLACKPILL.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "CHAL_GAYA_BHAI_BLACKPILL.hex"
	@echo 'Finished building: $@'
	@echo ' '

CHAL_GAYA_BHAI_BLACKPILL.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CHAL_GAYA_BHAI_BLACKPILL.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CHAL_GAYA_BHAI_BLACKPILL.bin CHAL_GAYA_BHAI_BLACKPILL.elf CHAL_GAYA_BHAI_BLACKPILL.hex CHAL_GAYA_BHAI_BLACKPILL.list CHAL_GAYA_BHAI_BLACKPILL.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/vl53l0x/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lsm6dsl/subdir.mk
-include Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lps22hb/subdir.mk
-include Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/lis3mdl/subdir.mk
-include Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/Components/hts221/subdir.mk
-include Drivers/BSP_B-L475E-IOT01-bfe8272ced90/Drivers/BSP/B-L475E-IOT01/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
FireSystem.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FireSystem.list \

OBJCOPY_BIN += \
FireSystem.bin \


# All Target
all: FireSystem.elf secondary-outputs

# Tool invocations
FireSystem.elf: $(OBJS) $(USER_OBJS) C:\Users\claud\STM32CubeIDE\workspace_1.1.0\FireSystem\STM32L475VGTX_FLASH.ld
	arm-none-eabi-g++ -o "FireSystem.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\claud\STM32CubeIDE\workspace_1.1.0\FireSystem\STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FireSystem.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FireSystem.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FireSystem.list"
	@echo 'Finished building: $@'
	@echo ' '

FireSystem.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FireSystem.bin"
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

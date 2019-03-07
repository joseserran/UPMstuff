################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include HAL_Driver/Src/Legacy/subdir.mk
-include HAL_Driver/Src/subdir.mk
-include BSP/STM32F411E-Discovery/subdir.mk
-include BSP/Components/wm8994/subdir.mk
-include BSP/Components/ts3510/subdir.mk
-include BSP/Components/stmpe811/subdir.mk
-include BSP/Components/stmpe1600/subdir.mk
-include BSP/Components/st7789h2/subdir.mk
-include BSP/Components/st7735/subdir.mk
-include BSP/Components/s5k5cag/subdir.mk
-include BSP/Components/ov2640/subdir.mk
-include BSP/Components/otm8009a/subdir.mk
-include BSP/Components/mfxstm32l152/subdir.mk
-include BSP/Components/lsm303dlhc/subdir.mk
-include BSP/Components/ls016b8uy/subdir.mk
-include BSP/Components/lis3dsh/subdir.mk
-include BSP/Components/lis302dl/subdir.mk
-include BSP/Components/l3gd20/subdir.mk
-include BSP/Components/ili9341/subdir.mk
-include BSP/Components/ili9325/subdir.mk
-include BSP/Components/ft6x06/subdir.mk
-include BSP/Components/exc7200/subdir.mk
-include BSP/Components/cs43l22/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: p4.elf

# Tool invocations
p4.elf: $(OBJS) $(USER_OBJS) C:\Users\startic\Dropbox\aaa\ Mi\ MUISE\ 2018-2019\LSEL\ -\ Laboratorio\ sistemas\ electronicos\practicas\practica4\p4_workspace\workspace\p4/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "p4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "p4.elf" "p4.bin"
	arm-none-eabi-size "p4.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

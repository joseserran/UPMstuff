################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/HAL_Driver/Inc/Legacy" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/inc" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/CMSIS/device" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/CMSIS/core" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/HAL_Driver/Inc" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/BSP/STM32F411E-Discovery/" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/BSP/Components" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Components/otm8009a/otm8009a.c 

OBJS += \
./BSP/Components/otm8009a/otm8009a.o 

C_DEPS += \
./BSP/Components/otm8009a/otm8009a.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Components/otm8009a/%.o: ../BSP/Components/otm8009a/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F411VETx -DDEBUG -DSTM32F411xE -DUSE_HAL_DRIVER -DUSE_RTOS_SYSTICK -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/HAL_Driver/Inc/Legacy" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/inc" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/CMSIS/device" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/CMSIS/core" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/HAL_Driver/Inc" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/BSP/STM32F411E-Discovery/" -I"C:/Users/startic/Dropbox/aaa Mi MUISE 2018-2019/LSEL - Laboratorio sistemas electronicos/practicas/practica4/p4_workspace/workspace/p4/BSP/Components" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.c 

OBJS += \
./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.o 

C_DEPS += \
./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP-main/ComputeLibrary/Source/%.o Drivers/CMSIS-DSP-main/ComputeLibrary/Source/%.su Drivers/CMSIS-DSP-main/ComputeLibrary/Source/%.cyclo: ../Drivers/CMSIS-DSP-main/ComputeLibrary/Source/%.c Drivers/CMSIS-DSP-main/ComputeLibrary/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/CMSIS-DSP-main/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-ComputeLibrary-2f-Source

clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-ComputeLibrary-2f-Source:
	-$(RM) ./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.cyclo ./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.d ./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.o ./Drivers/CMSIS-DSP-main/ComputeLibrary/Source/arm_cl_tables.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-ComputeLibrary-2f-Source


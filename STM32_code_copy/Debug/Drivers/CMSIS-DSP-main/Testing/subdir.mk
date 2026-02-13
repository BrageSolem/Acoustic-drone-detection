################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP-main/Testing/patterndata.c 

OBJS += \
./Drivers/CMSIS-DSP-main/Testing/patterndata.o 

C_DEPS += \
./Drivers/CMSIS-DSP-main/Testing/patterndata.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP-main/Testing/%.o Drivers/CMSIS-DSP-main/Testing/%.su Drivers/CMSIS-DSP-main/Testing/%.cyclo: ../Drivers/CMSIS-DSP-main/Testing/%.c Drivers/CMSIS-DSP-main/Testing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing

clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing:
	-$(RM) ./Drivers/CMSIS-DSP-main/Testing/patterndata.cyclo ./Drivers/CMSIS-DSP-main/Testing/patterndata.d ./Drivers/CMSIS-DSP-main/Testing/patterndata.o ./Drivers/CMSIS-DSP-main/Testing/patterndata.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing


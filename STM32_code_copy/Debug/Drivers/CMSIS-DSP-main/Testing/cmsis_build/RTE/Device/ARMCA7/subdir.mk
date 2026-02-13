################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.c \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.c \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.c 

OBJS += \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.o \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.o \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.o 

C_DEPS += \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.d \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.d \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/%.o Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/%.su Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/%.cyclo: ../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/%.c Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCA7

clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCA7:
	-$(RM) ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.cyclo ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.d ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.o ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/mmu_ARMCA7.su ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.cyclo ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.d ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.o ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/startup_ARMCA7.su ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.cyclo ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.d ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.o ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCA7/system_ARMCA7.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCA7


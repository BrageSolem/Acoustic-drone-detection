################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.c \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.c 

S_UPPER_SRCS += \
../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.S 

OBJS += \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.o \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.o 

S_UPPER_DEPS += \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.d 

C_DEPS += \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.d \
./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.o: ../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.S Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.o Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.su Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.cyclo: ../Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/%.c Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM35P_DSP_FP

clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM35P_DSP_FP:
	-$(RM) ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.cyclo ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.d ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.o ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/startup_ARMCM35P.su ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.cyclo ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.d ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.o ./Drivers/CMSIS-DSP-main/Testing/cmsis_build/RTE/Device/ARMCM35P_DSP_FP/system_ARMCM35P.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Testing-2f-cmsis_build-2f-RTE-2f-Device-2f-ARMCM35P_DSP_FP


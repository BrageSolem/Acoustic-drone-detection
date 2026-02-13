################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PDM2PCM/App/pdm2pcm.c 

OBJS += \
./PDM2PCM/App/pdm2pcm.o 

C_DEPS += \
./PDM2PCM/App/pdm2pcm.d 


# Each subdirectory must supply rules for building sources it contributes
PDM2PCM/App/%.o PDM2PCM/App/%.su PDM2PCM/App/%.cyclo: ../PDM2PCM/App/%.c PDM2PCM/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Drivers/Middlewares/ST/PDM2PCM/Inc -I../Core/Inc -I../Drivers/CMSIS-DSP-main/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PDM2PCM-2f-App

clean-PDM2PCM-2f-App:
	-$(RM) ./PDM2PCM/App/pdm2pcm.cyclo ./PDM2PCM/App/pdm2pcm.d ./PDM2PCM/App/pdm2pcm.o ./PDM2PCM/App/pdm2pcm.su

.PHONY: clean-PDM2PCM-2f-App


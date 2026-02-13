################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.c \
../Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.c 

OBJS += \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.o \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.o 

C_DEPS += \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.d \
./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS-DSP-main/Source/WindowFunctions/%.o Drivers/CMSIS-DSP-main/Source/WindowFunctions/%.su Drivers/CMSIS-DSP-main/Source/WindowFunctions/%.cyclo: ../Drivers/CMSIS-DSP-main/Source/WindowFunctions/%.c Drivers/CMSIS-DSP-main/Source/WindowFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/CMSIS-DSP-main/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Source-2f-WindowFunctions

clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Source-2f-WindowFunctions:
	-$(RM) ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/WindowFunctions.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f32.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_bartlett_f64.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f32.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_blackman_harris_92db_f64.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f32.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hamming_f64.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f32.su ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.cyclo ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.d ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.o ./Drivers/CMSIS-DSP-main/Source/WindowFunctions/arm_hanning_f64.su

.PHONY: clean-Drivers-2f-CMSIS-2d-DSP-2d-main-2f-Source-2f-WindowFunctions


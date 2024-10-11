################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SCSLib/Src/SCS.c \
../Drivers/SCSLib/Src/SCSerail.c \
../Drivers/SCSLib/Src/SMS_STS.c \
../Drivers/SCSLib/Src/uart.c 

OBJS += \
./Drivers/SCSLib/Src/SCS.o \
./Drivers/SCSLib/Src/SCSerail.o \
./Drivers/SCSLib/Src/SMS_STS.o \
./Drivers/SCSLib/Src/uart.o 

C_DEPS += \
./Drivers/SCSLib/Src/SCS.d \
./Drivers/SCSLib/Src/SCSerail.d \
./Drivers/SCSLib/Src/SMS_STS.d \
./Drivers/SCSLib/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SCSLib/Src/%.o Drivers/SCSLib/Src/%.su Drivers/SCSLib/Src/%.cyclo: ../Drivers/SCSLib/Src/%.c Drivers/SCSLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"/home/blumenkratz/.config/STM32CubeIDE/workspace_1.16.1/STM32 Motion Tracking Mount/Drivers/SCSLib/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SCSLib-2f-Src

clean-Drivers-2f-SCSLib-2f-Src:
	-$(RM) ./Drivers/SCSLib/Src/SCS.cyclo ./Drivers/SCSLib/Src/SCS.d ./Drivers/SCSLib/Src/SCS.o ./Drivers/SCSLib/Src/SCS.su ./Drivers/SCSLib/Src/SCSerail.cyclo ./Drivers/SCSLib/Src/SCSerail.d ./Drivers/SCSLib/Src/SCSerail.o ./Drivers/SCSLib/Src/SCSerail.su ./Drivers/SCSLib/Src/SMS_STS.cyclo ./Drivers/SCSLib/Src/SMS_STS.d ./Drivers/SCSLib/Src/SMS_STS.o ./Drivers/SCSLib/Src/SMS_STS.su ./Drivers/SCSLib/Src/uart.cyclo ./Drivers/SCSLib/Src/uart.d ./Drivers/SCSLib/Src/uart.o ./Drivers/SCSLib/Src/uart.su

.PHONY: clean-Drivers-2f-SCSLib-2f-Src


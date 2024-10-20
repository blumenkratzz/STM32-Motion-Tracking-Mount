################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SCSLib/SCS.c \
../Drivers/SCSLib/SCSerail.c \
../Drivers/SCSLib/SMS_STS.c \
../Drivers/SCSLib/uart.c 

OBJS += \
./Drivers/SCSLib/SCS.o \
./Drivers/SCSLib/SCSerail.o \
./Drivers/SCSLib/SMS_STS.o \
./Drivers/SCSLib/uart.o 

C_DEPS += \
./Drivers/SCSLib/SCS.d \
./Drivers/SCSLib/SCSerail.d \
./Drivers/SCSLib/SMS_STS.d \
./Drivers/SCSLib/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SCSLib/%.o Drivers/SCSLib/%.su Drivers/SCSLib/%.cyclo: ../Drivers/SCSLib/%.c Drivers/SCSLib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"${workspace_loc:/STM32 Motion Tracking Mount/Drivers/SCSLib" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SCSLib

clean-Drivers-2f-SCSLib:
	-$(RM) ./Drivers/SCSLib/SCS.cyclo ./Drivers/SCSLib/SCS.d ./Drivers/SCSLib/SCS.o ./Drivers/SCSLib/SCS.su ./Drivers/SCSLib/SCSerail.cyclo ./Drivers/SCSLib/SCSerail.d ./Drivers/SCSLib/SCSerail.o ./Drivers/SCSLib/SCSerail.su ./Drivers/SCSLib/SMS_STS.cyclo ./Drivers/SCSLib/SMS_STS.d ./Drivers/SCSLib/SMS_STS.o ./Drivers/SCSLib/SMS_STS.su ./Drivers/SCSLib/uart.cyclo ./Drivers/SCSLib/uart.d ./Drivers/SCSLib/uart.o ./Drivers/SCSLib/uart.su

.PHONY: clean-Drivers-2f-SCSLib


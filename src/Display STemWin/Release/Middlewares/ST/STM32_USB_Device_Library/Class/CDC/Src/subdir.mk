################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o: ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Inc" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Drivers/STM32F7xx_HAL_Driver/Inc" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Drivers/CMSIS/Include" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"D:/za_konsultacije/touch-screen-with-stemwin/Touch Screen with STemWin/Display STemWin/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



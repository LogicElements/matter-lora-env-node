################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BQ25185.c \
../Core/Src/DEV_Config.c \
../Core/Src/EPD_2in13_V4.c \
../Core/Src/EPD_2in13_V4_test.c \
../Core/Src/GUI.c \
../Core/Src/GUI_Paint.c \
../Core/Src/ImageData.c \
../Core/Src/ImageData2.c \
../Core/Src/RTC.c \
../Core/Src/SDC41.c \
../Core/Src/SGP40.c \
../Core/Src/SHT41.c \
../Core/Src/WioE5.c \
../Core/Src/app.c \
../Core/Src/button_handler.c \
../Core/Src/conf_console.c \
../Core/Src/ee_config.c \
../Core/Src/esp32c6.c \
../Core/Src/gpio_sleep.c \
../Core/Src/images.c \
../Core/Src/m24c02.c \
../Core/Src/main.c \
../Core/Src/qrcodegen.c \
../Core/Src/sensirion_gas_index_algorithm.c \
../Core/Src/stm32u0xx_hal_msp.c \
../Core/Src/stm32u0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u0xx.c 

OBJS += \
./Core/Src/BQ25185.o \
./Core/Src/DEV_Config.o \
./Core/Src/EPD_2in13_V4.o \
./Core/Src/EPD_2in13_V4_test.o \
./Core/Src/GUI.o \
./Core/Src/GUI_Paint.o \
./Core/Src/ImageData.o \
./Core/Src/ImageData2.o \
./Core/Src/RTC.o \
./Core/Src/SDC41.o \
./Core/Src/SGP40.o \
./Core/Src/SHT41.o \
./Core/Src/WioE5.o \
./Core/Src/app.o \
./Core/Src/button_handler.o \
./Core/Src/conf_console.o \
./Core/Src/ee_config.o \
./Core/Src/esp32c6.o \
./Core/Src/gpio_sleep.o \
./Core/Src/images.o \
./Core/Src/m24c02.o \
./Core/Src/main.o \
./Core/Src/qrcodegen.o \
./Core/Src/sensirion_gas_index_algorithm.o \
./Core/Src/stm32u0xx_hal_msp.o \
./Core/Src/stm32u0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u0xx.o 

C_DEPS += \
./Core/Src/BQ25185.d \
./Core/Src/DEV_Config.d \
./Core/Src/EPD_2in13_V4.d \
./Core/Src/EPD_2in13_V4_test.d \
./Core/Src/GUI.d \
./Core/Src/GUI_Paint.d \
./Core/Src/ImageData.d \
./Core/Src/ImageData2.d \
./Core/Src/RTC.d \
./Core/Src/SDC41.d \
./Core/Src/SGP40.d \
./Core/Src/SHT41.d \
./Core/Src/WioE5.d \
./Core/Src/app.d \
./Core/Src/button_handler.d \
./Core/Src/conf_console.d \
./Core/Src/ee_config.d \
./Core/Src/esp32c6.d \
./Core/Src/gpio_sleep.d \
./Core/Src/images.d \
./Core/Src/m24c02.d \
./Core/Src/main.d \
./Core/Src/qrcodegen.d \
./Core/Src/sensirion_gas_index_algorithm.d \
./Core/Src/stm32u0xx_hal_msp.d \
./Core/Src/stm32u0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U073xx -c -I../Core/Inc -I../Drivers/STM32U0xx_HAL_Driver/Inc -I../Drivers/STM32U0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BQ25185.cyclo ./Core/Src/BQ25185.d ./Core/Src/BQ25185.o ./Core/Src/BQ25185.su ./Core/Src/DEV_Config.cyclo ./Core/Src/DEV_Config.d ./Core/Src/DEV_Config.o ./Core/Src/DEV_Config.su ./Core/Src/EPD_2in13_V4.cyclo ./Core/Src/EPD_2in13_V4.d ./Core/Src/EPD_2in13_V4.o ./Core/Src/EPD_2in13_V4.su ./Core/Src/EPD_2in13_V4_test.cyclo ./Core/Src/EPD_2in13_V4_test.d ./Core/Src/EPD_2in13_V4_test.o ./Core/Src/EPD_2in13_V4_test.su ./Core/Src/GUI.cyclo ./Core/Src/GUI.d ./Core/Src/GUI.o ./Core/Src/GUI.su ./Core/Src/GUI_Paint.cyclo ./Core/Src/GUI_Paint.d ./Core/Src/GUI_Paint.o ./Core/Src/GUI_Paint.su ./Core/Src/ImageData.cyclo ./Core/Src/ImageData.d ./Core/Src/ImageData.o ./Core/Src/ImageData.su ./Core/Src/ImageData2.cyclo ./Core/Src/ImageData2.d ./Core/Src/ImageData2.o ./Core/Src/ImageData2.su ./Core/Src/RTC.cyclo ./Core/Src/RTC.d ./Core/Src/RTC.o ./Core/Src/RTC.su ./Core/Src/SDC41.cyclo ./Core/Src/SDC41.d ./Core/Src/SDC41.o ./Core/Src/SDC41.su ./Core/Src/SGP40.cyclo ./Core/Src/SGP40.d ./Core/Src/SGP40.o ./Core/Src/SGP40.su ./Core/Src/SHT41.cyclo ./Core/Src/SHT41.d ./Core/Src/SHT41.o ./Core/Src/SHT41.su ./Core/Src/WioE5.cyclo ./Core/Src/WioE5.d ./Core/Src/WioE5.o ./Core/Src/WioE5.su ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/button_handler.cyclo ./Core/Src/button_handler.d ./Core/Src/button_handler.o ./Core/Src/button_handler.su ./Core/Src/conf_console.cyclo ./Core/Src/conf_console.d ./Core/Src/conf_console.o ./Core/Src/conf_console.su ./Core/Src/ee_config.cyclo ./Core/Src/ee_config.d ./Core/Src/ee_config.o ./Core/Src/ee_config.su ./Core/Src/esp32c6.cyclo ./Core/Src/esp32c6.d ./Core/Src/esp32c6.o ./Core/Src/esp32c6.su ./Core/Src/gpio_sleep.cyclo ./Core/Src/gpio_sleep.d ./Core/Src/gpio_sleep.o ./Core/Src/gpio_sleep.su ./Core/Src/images.cyclo ./Core/Src/images.d ./Core/Src/images.o ./Core/Src/images.su ./Core/Src/m24c02.cyclo ./Core/Src/m24c02.d ./Core/Src/m24c02.o ./Core/Src/m24c02.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/qrcodegen.cyclo ./Core/Src/qrcodegen.d ./Core/Src/qrcodegen.o ./Core/Src/qrcodegen.su ./Core/Src/sensirion_gas_index_algorithm.cyclo ./Core/Src/sensirion_gas_index_algorithm.d ./Core/Src/sensirion_gas_index_algorithm.o ./Core/Src/sensirion_gas_index_algorithm.su ./Core/Src/stm32u0xx_hal_msp.cyclo ./Core/Src/stm32u0xx_hal_msp.d ./Core/Src/stm32u0xx_hal_msp.o ./Core/Src/stm32u0xx_hal_msp.su ./Core/Src/stm32u0xx_it.cyclo ./Core/Src/stm32u0xx_it.d ./Core/Src/stm32u0xx_it.o ./Core/Src/stm32u0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u0xx.cyclo ./Core/Src/system_stm32u0xx.d ./Core/Src/system_stm32u0xx.o ./Core/Src/system_stm32u0xx.su

.PHONY: clean-Core-2f-Src


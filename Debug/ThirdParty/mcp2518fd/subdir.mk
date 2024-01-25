################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/mcp2518fd/drv_canfdspi_api.c \
../ThirdParty/mcp2518fd/drv_spi.c 

OBJS += \
./ThirdParty/mcp2518fd/drv_canfdspi_api.o \
./ThirdParty/mcp2518fd/drv_spi.o 

C_DEPS += \
./ThirdParty/mcp2518fd/drv_canfdspi_api.d \
./ThirdParty/mcp2518fd/drv_spi.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/mcp2518fd/%.o ThirdParty/mcp2518fd/%.su ThirdParty/mcp2518fd/%.cyclo: ../ThirdParty/mcp2518fd/%.c ThirdParty/mcp2518fd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/vieth/Documents/stm32/stm32_canfd/stm32f407_w_mcp2518fd-main/stm32f407_w_mcp2518fd-main/ThirdParty/mcp2518fd" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-mcp2518fd

clean-ThirdParty-2f-mcp2518fd:
	-$(RM) ./ThirdParty/mcp2518fd/drv_canfdspi_api.cyclo ./ThirdParty/mcp2518fd/drv_canfdspi_api.d ./ThirdParty/mcp2518fd/drv_canfdspi_api.o ./ThirdParty/mcp2518fd/drv_canfdspi_api.su ./ThirdParty/mcp2518fd/drv_spi.cyclo ./ThirdParty/mcp2518fd/drv_spi.d ./ThirdParty/mcp2518fd/drv_spi.o ./ThirdParty/mcp2518fd/drv_spi.su

.PHONY: clean-ThirdParty-2f-mcp2518fd


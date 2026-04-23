################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MLX90640/Src/MLX90640_API.c \
../Drivers/MLX90640/Src/MLX90640_I2C_Driver.c \
../Drivers/MLX90640/Src/i2c.c 

OBJS += \
./Drivers/MLX90640/Src/MLX90640_API.o \
./Drivers/MLX90640/Src/MLX90640_I2C_Driver.o \
./Drivers/MLX90640/Src/i2c.o 

C_DEPS += \
./Drivers/MLX90640/Src/MLX90640_API.d \
./Drivers/MLX90640/Src/MLX90640_I2C_Driver.d \
./Drivers/MLX90640/Src/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MLX90640/Src/%.o Drivers/MLX90640/Src/%.su Drivers/MLX90640/Src/%.cyclo: ../Drivers/MLX90640/Src/%.c Drivers/MLX90640/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MLX90640-2f-Src

clean-Drivers-2f-MLX90640-2f-Src:
	-$(RM) ./Drivers/MLX90640/Src/MLX90640_API.cyclo ./Drivers/MLX90640/Src/MLX90640_API.d ./Drivers/MLX90640/Src/MLX90640_API.o ./Drivers/MLX90640/Src/MLX90640_API.su ./Drivers/MLX90640/Src/MLX90640_I2C_Driver.cyclo ./Drivers/MLX90640/Src/MLX90640_I2C_Driver.d ./Drivers/MLX90640/Src/MLX90640_I2C_Driver.o ./Drivers/MLX90640/Src/MLX90640_I2C_Driver.su ./Drivers/MLX90640/Src/i2c.cyclo ./Drivers/MLX90640/Src/i2c.d ./Drivers/MLX90640/Src/i2c.o ./Drivers/MLX90640/Src/i2c.su

.PHONY: clean-Drivers-2f-MLX90640-2f-Src


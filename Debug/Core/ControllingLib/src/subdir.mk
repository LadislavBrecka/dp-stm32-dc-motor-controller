################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/ControllingLib/src/closed_loop.cpp \
../Core/ControllingLib/src/identification.cpp \
../Core/ControllingLib/src/transfer_fcn.cpp 

OBJS += \
./Core/ControllingLib/src/closed_loop.o \
./Core/ControllingLib/src/identification.o \
./Core/ControllingLib/src/transfer_fcn.o 

CPP_DEPS += \
./Core/ControllingLib/src/closed_loop.d \
./Core/ControllingLib/src/identification.d \
./Core/ControllingLib/src/transfer_fcn.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ControllingLib/src/%.o Core/ControllingLib/src/%.su: ../Core/ControllingLib/src/%.cpp Core/ControllingLib/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"/home/ladislav/School/dp/stm32-projects/demo-pwm/Core/ControllingLib/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-ControllingLib-2f-src

clean-Core-2f-ControllingLib-2f-src:
	-$(RM) ./Core/ControllingLib/src/closed_loop.d ./Core/ControllingLib/src/closed_loop.o ./Core/ControllingLib/src/closed_loop.su ./Core/ControllingLib/src/identification.d ./Core/ControllingLib/src/identification.o ./Core/ControllingLib/src/identification.su ./Core/ControllingLib/src/transfer_fcn.d ./Core/ControllingLib/src/transfer_fcn.o ./Core/ControllingLib/src/transfer_fcn.su

.PHONY: clean-Core-2f-ControllingLib-2f-src


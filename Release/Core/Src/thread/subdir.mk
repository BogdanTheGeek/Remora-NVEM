################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/thread/pruThread.cpp \
../Core/Src/thread/timer.cpp \
../Core/Src/thread/timerInterrupt.cpp 

OBJS += \
./Core/Src/thread/pruThread.o \
./Core/Src/thread/timer.o \
./Core/Src/thread/timerInterrupt.o 

CPP_DEPS += \
./Core/Src/thread/pruThread.d \
./Core/Src/thread/timer.d \
./Core/Src/thread/timerInterrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/thread/%.o: ../Core/Src/thread/%.cpp Core/Src/thread/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/posix -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/system/arch -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-thread

clean-Core-2f-Src-2f-thread:
	-$(RM) ./Core/Src/thread/pruThread.d ./Core/Src/thread/pruThread.o ./Core/Src/thread/timer.d ./Core/Src/thread/timer.o ./Core/Src/thread/timerInterrupt.d ./Core/Src/thread/timerInterrupt.o

.PHONY: clean-Core-2f-Src-2f-thread


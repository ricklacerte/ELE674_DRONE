################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/DroneFirmware.c \
../src/Motor.c 

OBJS += \
./src/DroneFirmware.o \
./src/Motor.o 

C_DEPS += \
./src/DroneFirmware.d \
./src/Motor.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-linux-gnueabi-gcc -O0 -g3 -Wall -lm -pthread -lrt -c -ftest-coverage -fprofile-arcs -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ELE674-Lab3

# Tool invocations
ELE674-Lab3: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	arm-none-linux-gnueabi-gcc -L/home/ens/AK22070/Documents/ELE674/Lab2-Image_Drone/CodeSourcery/arm-none-linux-gnueabi/libc/usr/lib -pthread -lm -lrt -shared -o "ELE674-Lab3" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(LIBRARIES) ELE674-Lab3
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

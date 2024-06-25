################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../shared/runtime/gchelper_generic.c \
../shared/runtime/pyexec.c \
../shared/runtime/stdout_helpers.c 

C_DEPS += \
./shared/runtime/gchelper_generic.d \
./shared/runtime/pyexec.d \
./shared/runtime/stdout_helpers.d 

OBJS += \
./shared/runtime/gchelper_generic.o \
./shared/runtime/pyexec.o \
./shared/runtime/stdout_helpers.o 


# Each subdirectory must supply rules for building sources it contributes
shared/runtime/%.o: ../shared/runtime/%.c shared/runtime/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv64-zephyr-elf-gcc -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g3 -I/home/dinesha/workarea/github/micropython/ports/riscduino/bsp/inc -I/home/dinesha/workarea/github/micropython/py -I/home/dinesha/workarea/github/micropython/ports/riscduino/build -I/home/dinesha/workarea/github/micropython/shared/readline -I/home/dinesha/workarea/github/micropython/shared/runtime -I/home/dinesha/workarea/github/micropython -I/home/dinesha/workarea/github/micropython/ports/riscduino -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



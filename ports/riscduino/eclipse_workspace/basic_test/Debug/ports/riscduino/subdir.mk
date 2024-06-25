################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ports/riscduino/main.c \
../ports/riscduino/mphalport.c 

C_DEPS += \
./ports/riscduino/main.d \
./ports/riscduino/mphalport.d 

OBJS += \
./ports/riscduino/main.o \
./ports/riscduino/mphalport.o 


# Each subdirectory must supply rules for building sources it contributes
ports/riscduino/%.o: ../ports/riscduino/%.c ports/riscduino/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv64-zephyr-elf-gcc -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g3 -I/home/dinesha/workarea/github/micropython/ports/riscduino/bsp/inc -I/home/dinesha/workarea/github/micropython/py -I/home/dinesha/workarea/github/micropython/ports/riscduino/build -I/home/dinesha/workarea/github/micropython/shared/readline -I/home/dinesha/workarea/github/micropython/shared/runtime -I/home/dinesha/workarea/github/micropython -I/home/dinesha/workarea/github/micropython/ports/riscduino -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



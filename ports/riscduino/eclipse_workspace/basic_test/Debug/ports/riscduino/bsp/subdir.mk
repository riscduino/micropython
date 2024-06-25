################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../ports/riscduino/bsp/startup.S 

OBJS += \
./ports/riscduino/bsp/startup.o 

S_UPPER_DEPS += \
./ports/riscduino/bsp/startup.d 


# Each subdirectory must supply rules for building sources it contributes
ports/riscduino/bsp/%.o: ../ports/riscduino/bsp/%.S ports/riscduino/bsp/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross Assembler'
	riscv64-zephyr-elf-gcc -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g3 -x assembler-with-cpp -I/home/dinesha/workarea/github/micropython/ports/riscduino/bsp/inc -I/home/dinesha/workarea/github/micropython/ports/riscduino/build -I/home/dinesha/workarea/github/micropython/shared/readline -I/home/dinesha/workarea/github/micropython -I/home/dinesha/workarea/github/micropython/shared/runtime -I/home/dinesha/workarea/github/micropython/ports/riscduino -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



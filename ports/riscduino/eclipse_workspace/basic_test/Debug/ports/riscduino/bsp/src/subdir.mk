################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ports/riscduino/bsp/src/core_riscv.c \
../ports/riscduino/bsp/src/debug.c \
../ports/riscduino/bsp/src/spp32r20xx_gpio.c \
../ports/riscduino/bsp/src/spp32r20xx_i2c.c \
../ports/riscduino/bsp/src/spp32r20xx_it.c \
../ports/riscduino/bsp/src/spp32r20xx_misc.c \
../ports/riscduino/bsp/src/spp32r20xx_rcc.c \
../ports/riscduino/bsp/src/spp32r20xx_rtc.c \
../ports/riscduino/bsp/src/spp32r20xx_spi.c \
../ports/riscduino/bsp/src/spp32r20xx_tim.c \
../ports/riscduino/bsp/src/spp32r20xx_usart.c \
../ports/riscduino/bsp/src/system_spp32r20xx.c 

C_DEPS += \
./ports/riscduino/bsp/src/core_riscv.d \
./ports/riscduino/bsp/src/debug.d \
./ports/riscduino/bsp/src/spp32r20xx_gpio.d \
./ports/riscduino/bsp/src/spp32r20xx_i2c.d \
./ports/riscduino/bsp/src/spp32r20xx_it.d \
./ports/riscduino/bsp/src/spp32r20xx_misc.d \
./ports/riscduino/bsp/src/spp32r20xx_rcc.d \
./ports/riscduino/bsp/src/spp32r20xx_rtc.d \
./ports/riscduino/bsp/src/spp32r20xx_spi.d \
./ports/riscduino/bsp/src/spp32r20xx_tim.d \
./ports/riscduino/bsp/src/spp32r20xx_usart.d \
./ports/riscduino/bsp/src/system_spp32r20xx.d 

OBJS += \
./ports/riscduino/bsp/src/core_riscv.o \
./ports/riscduino/bsp/src/debug.o \
./ports/riscduino/bsp/src/spp32r20xx_gpio.o \
./ports/riscduino/bsp/src/spp32r20xx_i2c.o \
./ports/riscduino/bsp/src/spp32r20xx_it.o \
./ports/riscduino/bsp/src/spp32r20xx_misc.o \
./ports/riscduino/bsp/src/spp32r20xx_rcc.o \
./ports/riscduino/bsp/src/spp32r20xx_rtc.o \
./ports/riscduino/bsp/src/spp32r20xx_spi.o \
./ports/riscduino/bsp/src/spp32r20xx_tim.o \
./ports/riscduino/bsp/src/spp32r20xx_usart.o \
./ports/riscduino/bsp/src/system_spp32r20xx.o 


# Each subdirectory must supply rules for building sources it contributes
ports/riscduino/bsp/src/%.o: ../ports/riscduino/bsp/src/%.c ports/riscduino/bsp/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv64-zephyr-elf-gcc -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g3 -I/home/dinesha/workarea/github/micropython/ports/riscduino/bsp/inc -I/home/dinesha/workarea/github/micropython/py -I/home/dinesha/workarea/github/micropython/ports/riscduino/build -I/home/dinesha/workarea/github/micropython/shared/readline -I/home/dinesha/workarea/github/micropython/shared/runtime -I/home/dinesha/workarea/github/micropython -I/home/dinesha/workarea/github/micropython/ports/riscduino -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



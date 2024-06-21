#Make Sure that Zypher Riscv cross compiler(riscv64-zephyr-elf-*) path in .bashrc
  export PATH=/home/<usernname>/zephyr-sdk-0.16.4/riscv64-zephyr-elf/bin/:$PATH
#if you want to change the cross compiler, then change CROSS_COMPILE in Makefile required name
CROSS_COMPILE ?= riscv64-zephyr-elf-
# To run the micro-python for riscduino, type
  make V=1

#to clean
  make clean

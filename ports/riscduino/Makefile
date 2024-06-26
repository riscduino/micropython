#SHELL = sh -xv
# Include the core environment definitions; this will set $(TOP).
CROSS_COMPILE ?= riscv64-zephyr-elf-
include ../../py/mkenv.mk

# Include py core make definitions.
include $(TOP)/py/py.mk

###############################################################################

# Project specific
BOARD ?= spp32r20xx
TARGET = $(BOARD).elf


SRC_DIR = bsp/src/
INC_DIR = bsp/inc/

# Linker Script
LD_SCRIPT = bsp/spp32r20xx.ld

# Set CFLAGS and libraries.
CFLAGS  = -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os
CFLAGS += -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g
CFLAGS += -std=gnu99
CFLAGS += -I. -I$(INC_DIR) -I$(BUILD) -I$(TOP)

# Linker Flags
LDFLAGS = -T $(LD_SCRIPT) -nostartfiles -Xlinker --gc-sections --specs=nano.specs --specs=nosys.specs
#OBJ DUMP
DUMPFLAGS = --source --all-headers --demangle --line-numbers --wide

#Verilog Copy
COPYFLAGS = -R .rel.dyn -O verilog

# Define the required object files.
OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_S:.S=.o))

# Define the required source files.
SRC_C = \
    main.c \
    mphalport.c \
    shared/readline/readline.c \
    shared/runtime/gchelper_generic.c \
    shared/runtime/pyexec.c \
    shared/runtime/stdout_helpers.c \

SRC_C += $(wildcard $(SRC_DIR)*.c) 
SRC_S += bsp/startup.S

# Define source files containung qstrs.
SRC_QSTR += shared/readline/readline.c shared/runtime/pyexec.c


# Define the top-level target, the main firmware.
all: $(BUILD)/$(TARGET)

# Define how to build the firmware.
$(BUILD)/$(TARGET): $(OBJ) 
	$(ECHO) "LINK $@" 
	#$(Q)$(CC) $(LDFLAGS) -o $@ $^ $(LIBS) 
	$(Q)$(CC) $(LDFLAGS) -o $@ $^
	$(Q)$(OBJDUMP) $(DUMPFLAGS) $@ > $(BUILD)/$(BOARD).dump
	$(Q)$(OBJCOPY) $(COPYFLAGS) $@  $(BUILD)/$(BOARD).hex
	$(Q)$(SIZE) $@

prog: $(BUILD)/$(TARGET)
	openocd -s openocd/  -f openocd/riscduino_d32a1.cfg -c "init" -c targets -c riscduino-halt -c targets -c 'riscduino-load build/bes32r20xx.elf' -c 'riscduino-reset' -c resume -c targets -d2

# Include remaining core make rules.
include $(TOP)/py/mkrules.mk


#SHELL = sh -xv
# Include the core environment definitions; this will set $(TOP).
CROSS_COMPILE ?= riscv64-zephyr-elf-
include ../../py/mkenv.mk

# Include py core make definitions.
include $(TOP)/py/py.mk

###############################################################################

# Project specific
BOARD ?= bes32r20xx
TARGET = $(BOARD).elf

SRC_DIR = bsp/src/
INC_DIR = bsp/inc/

# Linker Script
LD_SCRIPT = bsp/bes32r20xx.ld

# Set CFLAGS and libraries.
CFLAGS  = -march=rv32imc_zicsr -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os
CFLAGS += -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -g
CFLAGS += -std=gnu99
CFLAGS += -I. -I$(INC_DIR) -I$(BUILD) -I$(TOP)

# Linker Flags
LDFLAGS = -T $(LD_SCRIPT) -nostartfiles -Xlinker --gc-sections --specs=nano.specs --specs=nosys.specs
#OBJ DUMP
DUMPFLAGS = --source --all-headers --demangle --line-numbers --wide

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
SRC_S += bsp/startup_bes32r20xx.S

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
	$(Q)$(SIZE) $@

# Include remaining core make rules.
include $(TOP)/py/mkrules.mk


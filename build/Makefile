#==============================================================================
# centipede_robot/build/Makefile
# Build system for Centipede Robot firmware
#==============================================================================

# Tools
NASM    = nasm
LD      = ld
OBJCOPY = objcopy
SIZE    = size

# Directories
SRC_DIR     = ../src
INC_DIRS    = ../inc ../config
BUILD_DIR   = .
LINKER_DIR  = ../linker

# Include flags
INC_FLAGS = $(addprefix -I, $(INC_DIRS))

# NASM flags
NASM_FLAGS = -f elf32 -g $(INC_FLAGS)

# Linker flags
LD_FLAGS = -T $(LINKER_DIR)/link.ld -m elf_i386

# Source files (sorted by dependency order)
SRCS = \
    $(SRC_DIR)/main.asm \
    $(SRC_DIR)/init.asm \
    $(SRC_DIR)/servo_control.asm \
    $(SRC_DIR)/sensors.asm \
    $(SRC_DIR)/communication.asm \
    $(SRC_DIR)/kinematics.asm \
    $(SRC_DIR)/gait_engine.asm \
    $(SRC_DIR)/state_machine.asm \
    $(SRC_DIR)/debug.asm

# Object files
OBJS = $(SRCS:.asm=.o)

# Targets
TARGET = centipede_robot.elf
BINARY = centipede_robot.bin

# Default target
all: $(BINARY) size

# Link
$(TARGET): $(OBJS) $(LINKER_DIR)/link.ld
	@echo "Linking $@..."
	$(LD) $(LD_FLAGS) -o $@ $(OBJS)
	@echo "Done."

# Compile ASM to object
$(SRC_DIR)/%.o: $(SRC_DIR)/%.asm
	@echo "Assembling $<..."
	$(NASM) $(NASM_FLAGS) -o $@ $<

# Generate raw binary
$(BINARY): $(TARGET)
	@echo "Generating binary..."
	$(OBJCOPY) -O binary $< $@

# Size report
size: $(TARGET)
	@echo ""
	@echo "=== Size Report ==="
	$(SIZE) -A -x $(TARGET)

# Clean
clean:
	@echo "Cleaning..."
	rm -f $(OBJS) $(TARGET) $(BINARY)
	@echo "Done."

# Run in emulator (QEMU)
qemu: $(BINARY)
	@echo "Running in QEMU (i386)..."
	qemu-system-i386 -kernel $(BINARY) -serial stdio

# Debug with GDB
debug: $(TARGET)
	@echo "Starting QEMU with GDB server on port 1234..."
	qemu-system-i386 -s -S -kernel $(BINARY) -serial stdio &
	@echo "Connect GDB: gdb $(TARGET) -ex 'target remote localhost:1234'"

# List source files
sources:
	@echo "Source files:"
	@for src in $(SRCS); do echo "  $$src"; done

# Dependencies (automatic)
%.d: %.asm
	@$(NASM) -M $(NASM_FLAGS) -o $@ $< > $@

-include $(SRCS:.asm=.d)

.PHONY: all clean qemu debug size sources


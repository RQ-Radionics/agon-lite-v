# Makefile para programas de usuario ESP32-MOS
#
# Prerequisito: toolchain instalado via ESP-IDF
#   source ~/esp/esp-idf/export.sh
#
# Uso:
#   make                      → compila todos los .c para ESP32-P4 (RISC-V)
#   make helloworld           → compila solo helloworld.bin
#   make clean

# ── Target: ESP32-P4 RISC-V (único soportado) ────────────────────────────
TARGET  := esp32p4

# ── Directorios ──────────────────────────────────────────────────────────
SDK_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR  := $(SDK_DIR)../components/mos_api/include

# ── Toolchain & arch flags ────────────────────────────────────────────────
CC      = riscv32-esp-elf-gcc
AS      = riscv32-esp-elf-gcc
OBJCOPY = riscv32-esp-elf-objcopy
OBJDUMP = riscv32-esp-elf-objdump
ARCH_FLAGS = -march=rv32imafc -mabi=ilp32f -DCONFIG_IDF_TARGET_ESP32P4=1

# ESP32-P4: unified PSRAM address space (DBUS == IBUS).
# s_exec_arena is at 0x48000000 (from build/esp32-mos.map .ext_ram.bss)
MOS_EXEC_BASE ?= 0x48000000
MOS_DBUS_BASE ?= 0x48000000

LIBGCC_DIR := $(dir $(shell $(CC) $(ARCH_FLAGS) -print-libgcc-file-name))
LIBC_DIR   := $(dir $(shell $(CC) $(ARCH_FLAGS) -print-file-name=libc.a))

# ── Flags de compilación ─────────────────────────────────────────────────
CFLAGS = \
    $(ARCH_FLAGS) \
    -Os \
    -std=c11 \
    -nostdlib \
    -ffreestanding \
    -fno-builtin \
    -Wall \
    -Wextra \
    -I$(INC_DIR)

ASFLAGS = \
    $(ARCH_FLAGS) \
    -nostdlib

LDFLAGS = \
    -T$(SDK_DIR)mos_user.ld \
    -Wl,--defsym,MOS_EXEC_BASE=$(MOS_EXEC_BASE) \
    -Wl,--defsym,MOS_DBUS_BASE=$(MOS_DBUS_BASE) \
    -nostdlib \
    -Wl,--gc-sections

# libgcc provides software float routines (__divsf3, __mulsf3, etc.)
# libc provides memcpy/memset that RISC-V GCC emits implicitly for struct copies.
LIBGCC_FLAGS = -L$(LIBGCC_DIR) -L$(LIBC_DIR) -lc -lgcc

# ── Shared objects (linked into every program) ────────────────────────────
COMMON_SRCS := $(SDK_DIR)mos_vector.S
COMMON_OBJS := $(SDK_DIR)mos_vector.$(TARGET).o

# ── Reglas ───────────────────────────────────────────────────────────────
SRCS    := $(wildcard $(SDK_DIR)*.c)
BINS    := $(patsubst $(SDK_DIR)%.c,$(SDK_DIR)%.bin,$(SRCS))

.PHONY: all clean disasm

all: $(BINS)

# Compile common assembly
$(SDK_DIR)mos_vector.$(TARGET).o: $(SDK_DIR)mos_vector.S
	$(AS) $(ASFLAGS) -c -o $@ $<

# Compilar .c → .elf  (mos_vector.o linked first so .text.vector is at offset 0)
$(SDK_DIR)%.elf: $(SDK_DIR)%.c $(SDK_DIR)mos_user.ld $(COMMON_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-Map,$(SDK_DIR)$*.map \
	    -o $@ $(COMMON_OBJS) $< $(LIBGCC_FLAGS)

# ELF → binario plano, and copy to data/<target>/ for flash_data.sh
$(SDK_DIR)%.bin: $(SDK_DIR)%.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"
	@mkdir -p $(SDK_DIR)../data/$(TARGET)
	@cp $@ $(SDK_DIR)../data/$(TARGET)/$*.bin && echo "  → ../data/$(TARGET)/$*.bin (copied)"

# Desensamblar para depuración
disasm: $(patsubst $(SDK_DIR)%.c,$(SDK_DIR)%.elf,$(SRCS))
	for f in $^; do \
	    $(OBJDUMP) -d $$f > $${f%.elf}.asm; \
	    echo "  → $${f%.elf}.asm"; \
	done

clean:
	rm -f $(SDK_DIR)*.elf $(SDK_DIR)*.bin $(SDK_DIR)*.map $(SDK_DIR)*.asm $(SDK_DIR)*.o

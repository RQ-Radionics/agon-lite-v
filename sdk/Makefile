# Makefile para programas de usuario ESP32-MOS
#
# Prerequisito: toolchain Xtensa instalado via ESP-IDF
#   source ~/esp/esp-idf/export.sh
#
# Uso:
#   make              → compila todos los .c en este directorio
#   make helloworld   → compila solo helloworld.bin
#   make clean

# ── Toolchain ────────────────────────────────────────────────────────────────
CC      = xtensa-esp-elf-gcc
AS      = xtensa-esp-elf-gcc
OBJCOPY = xtensa-esp-elf-objcopy
OBJDUMP = xtensa-esp-elf-objdump

# ── Directorios ──────────────────────────────────────────────────────────────
SDK_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR  := $(SDK_DIR)../components/mos_api/include

# ── Dirección de carga en PSRAM ──────────────────────────────────────────────
# s_exec_arena DBUS (data window):        0x3C0E0000
# s_exec_arena IBUS (instruction window): 0x420E0000  (= DBUS + 0x6000000)
# Everything linked at IBUS; PMS disabled so l32r data reads from IBUS work.
MOS_EXEC_BASE ?= 0x420E0000

# ── Flags de compilación ─────────────────────────────────────────────────────
CFLAGS = \
    -mlongcalls \
    -mtext-section-literals \
    -Os \
    -std=c11 \
    -nostdlib \
    -ffreestanding \
    -fno-builtin \
    -Wall \
    -Wextra \
    -I$(INC_DIR)

ASFLAGS = \
    -mlongcalls \
    -nostdlib

LDFLAGS = \
    -T$(SDK_DIR)mos_user.ld \
    -DMOS_EXEC_BASE=$(MOS_EXEC_BASE) \
    -nostdlib \
    -Wl,--gc-sections

# ── Shared objects (linked into every program) ────────────────────────────────
COMMON_SRCS := $(SDK_DIR)mos_vector.S
COMMON_OBJS := $(SDK_DIR)mos_vector.o

# ── Reglas ───────────────────────────────────────────────────────────────────
SRCS    := $(wildcard $(SDK_DIR)*.c)
BINS    := $(patsubst $(SDK_DIR)%.c,$(SDK_DIR)%.bin,$(SRCS))

.PHONY: all clean disasm

all: $(BINS)

# Compile common assembly
$(SDK_DIR)mos_vector.o: $(SDK_DIR)mos_vector.S
	$(AS) $(ASFLAGS) -c -o $@ $<

# Compilar .c → .elf  (mos_vector.o linked first so .text.vector is at offset 0)
$(SDK_DIR)%.elf: $(SDK_DIR)%.c $(SDK_DIR)mos_user.ld $(COMMON_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-Map,$(SDK_DIR)$*.map \
	    -o $@ $(COMMON_OBJS) $<

# ELF → binario plano
$(SDK_DIR)%.bin: $(SDK_DIR)%.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

# Desensamblar para depuración
disasm: $(patsubst $(SDK_DIR)%.c,$(SDK_DIR)%.elf,$(SRCS))
	for f in $^; do \
	    $(OBJDUMP) -d $$f > $${f%.elf}.asm; \
	    echo "  → $${f%.elf}.asm"; \
	done

clean:
	rm -f $(SDK_DIR)*.elf $(SDK_DIR)*.bin $(SDK_DIR)*.map $(SDK_DIR)*.asm $(SDK_DIR)*.o

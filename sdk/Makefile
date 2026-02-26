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
OBJCOPY = xtensa-esp-elf-objcopy
OBJDUMP = xtensa-esp-elf-objdump

# ── Directorios ──────────────────────────────────────────────────────────────
SDK_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR  := $(SDK_DIR)../components/mos_api/include

# ── Dirección de carga en PSRAM ──────────────────────────────────────────────
# Debe coincidir con MOS_EXEC_BASE en mos_loader.h
MOS_EXEC_BASE ?= 0x3C0E0000

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

LDFLAGS = \
    -T$(SDK_DIR)mos_user.ld \
    -DMOS_EXEC_BASE=$(MOS_EXEC_BASE) \
    -nostdlib \
    -Wl,--gc-sections

# ── Reglas ───────────────────────────────────────────────────────────────────
SRCS    := $(wildcard *.c)
ELFS    := $(SRCS:.c=.elf)
BINS    := $(SRCS:.c=.bin)

.PHONY: all clean disasm

all: $(BINS)

# Compilar .c → .elf
%.elf: %.c $(SDK_DIR)mos_user.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-Map,$*.map -o $@ $<

# ELF → binario plano
%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

# Desensamblar para depuración
disasm: $(ELFS)
	for f in $^; do \
	    $(OBJDUMP) -d $$f > $${f%.elf}.asm; \
	    echo "  → $${f%.elf}.asm"; \
	done

clean:
	rm -f *.elf *.bin *.map *.asm

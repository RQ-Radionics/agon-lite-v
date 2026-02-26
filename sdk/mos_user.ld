/*
 * mos_user.ld - Linker script for ESP32-MOS user programs
 *
 * ESP32-S3 PSRAM virtual address windows (same physical pages):
 *   IBUS (0x42000000): instruction fetch + 32-bit data loads only
 *   DBUS (0x3C000000): byte/half/word data loads and stores
 *
 * Code (.text, .literal) → VMA in IBUS (0x420E0000+)
 *   l32r reads literal pool addresses via IBUS — 32-bit aligned, OK.
 *   Function calls via callx8 work from IBUS.
 *
 * Data (.rodata, .data) → VMA in DBUS (0x3C0E0000+)
 *   Strings need byte loads (l8ui) which only work via DBUS.
 *   LMA = VMA - DBUS_BASE + IBUS_BASE (= same physical byte in the flat binary)
 *
 * The flat binary produced by objcopy -O binary has all sections at
 * contiguous file offsets. Code fills 0x00..CODE_END, rodata follows
 * immediately. objcopy computes file offsets from LMA, so rodata LMA
 * must equal its natural continuation after .text in the file.
 *
 * Entry: byte 0 = 'j _start' trampoline (mos_vector.S)
 * Prototype: int _start(int argc, char **argv, t_mos_api *mos);
 */

ENTRY(_start)

MOS_EXEC_BASE = DEFINED(MOS_EXEC_BASE) ? MOS_EXEC_BASE : 0x420E0000;
MOS_DBUS_BASE = DEFINED(MOS_DBUS_BASE) ? MOS_DBUS_BASE : 0x3C0E0000;
BUS_OFFSET    = MOS_EXEC_BASE - MOS_DBUS_BASE;   /* 0x06000000 */

MEMORY {
    IBUS (rx)  : ORIGIN = MOS_EXEC_BASE, LENGTH = 2M
    DBUS (rw)  : ORIGIN = MOS_DBUS_BASE, LENGTH = 2M
}

SECTIONS {
    /* ── Instruction bus: code ─────────────────────────────────────────────── */
    . = MOS_EXEC_BASE;

    .text.vector : {
        KEEP(*(.text.vector))       /* byte 0: 'j _start' trampoline */
    } > IBUS

    .literal : {
        *(.literal .literal.* .text.entry.literal .text.*.literal)
    } > IBUS

    .text : {
        *(.text.entry)              /* _start */
        *(.text*)
    } > IBUS

    _ibus_end = .;
    _dbus_data_start = _ibus_end - BUS_OFFSET;

    /* ── Data bus: rodata + data (same physical bytes, DBUS VMA) ───────────── */
    /* Use explicit "VMA : AT(LMA)" form — the "> REGION" form resets VMA to
     * the region origin and ignores the current location counter. */

    .rodata _dbus_data_start : AT(_ibus_end) {
        _dbus_start = .;
        *(.rodata*)
        *(.rodata)
    }

    . = ALIGN(4);
    .data : {
        *(.data*)
    }

    . = ALIGN(4);
    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    }

    /DISCARD/ : { *(.comment) *(.note*) *(.ARM*) }
}

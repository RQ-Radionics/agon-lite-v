/*
 * mos_user.ld - Linker script for ESP32-MOS user programs
 *
 * Multi-target: parametrized via MOS_EXEC_BASE and MOS_DBUS_BASE.
 *
 * ESP32-S3 (Xtensa): separate IBUS/DBUS windows, BUS_OFFSET = 0x06000000
 *   IBUS (0x42000000): instruction fetch + 32-bit data loads only
 *   DBUS (0x3C000000): byte/half/word data loads and stores
 *   Code (.text) → IBUS VMA; Data (.rodata, .data) → DBUS VMA.
 *
 * ESP32-P4 (RISC-V): unified address space, BUS_OFFSET = 0
 *   IBUS == DBUS at 0x48xxxxxx. All sections share the same address space.
 *
 * The flat binary produced by objcopy -O binary has all sections at
 * contiguous file offsets. objcopy computes file offsets from LMA.
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
        *(.srodata*)          /* RISC-V small read-only data */
        . = ALIGN(4);
    }
    _rodata_lma_end = _ibus_end + SIZEOF(.rodata);

    /* .data: VMA in DBUS (for byte-accessible r/w), LMA continues after .rodata
     * in the flat binary (same file offset trick as .rodata). */
    .data ALIGN(_dbus_data_start + SIZEOF(.rodata), 4)
          : AT(ALIGN(_rodata_lma_end, 4)) {
        *(.data*)
        *(.sdata*)            /* RISC-V small data */
        . = ALIGN(4);
    }

    /* _data_end: DBUS VMA just after .data.  BSS follows here at runtime.
     * We must set VMA explicitly because NOLOAD sections do not advance
     * the location counter in the same way as LOAD sections. */
    _data_end = ALIGN(. , 4);

    .bss _data_end (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(.sbss*)             /* RISC-V small bss */
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    }

    /DISCARD/ : { *(.comment) *(.note*) *(.ARM*) }
}

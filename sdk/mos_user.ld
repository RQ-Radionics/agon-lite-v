/*
 * mos_user.ld - Linker script for ESP32-MOS user programs
 *
 * Produces a flat binary (objcopy -O binary) loaded into PSRAM by mos_loader.
 * The loader jumps to byte 0 of the binary.
 *
 * Layout at MOS_EXEC_BASE:
 *   [0x00] .text.vector  — 3-byte 'j _start' trampoline (byte 0 = entry)
 *   [0x03] .literal*     — l32r literal pools (must be within ±256KB of user)
 *   [aligned] .text.entry — _start
 *   [...] .text*, .rodata*, .data*
 *
 * Entry point: _start (jumped to via trampoline at byte 0)
 * Prototype:   int _start(int argc, char **argv, t_mos_api *mos);
 */

ENTRY(_start)

/* IBUS alias: dbus=0x3C0E0000 → ibus=0x420E0000 (CPU fetches code from IBUS) */
MOS_EXEC_BASE = DEFINED(MOS_EXEC_BASE) ? MOS_EXEC_BASE : 0x420E0000;

MEMORY {
    /* PSRAM instruction-bus window on ESP32-S3 (0x42000000-0x44000000).
     * CPU fetches code from here; data window is at 0x3C000000+same offset. */
    PSRAM (rwx) : ORIGIN = MOS_EXEC_BASE, LENGTH = 2M
}

SECTIONS {
    . = MOS_EXEC_BASE;

    /* Byte 0: jump trampoline — loader always jumps here.
     * KEEP so --gc-sections doesn't discard it (nothing calls it). */
    .text.vector : {
        KEEP(*(.text.vector))
    } > PSRAM

    /* Literal pools immediately after trampoline, before code */
    .literal : {
        *(.literal .literal.* .text.entry.literal .text.*.literal)
    } > PSRAM

    . = ALIGN(4);
    .text : {
        *(.text.entry)   /* _start */
        *(.text*)
        *(.rodata*)
    } > PSRAM

    . = ALIGN(4);
    .data : {
        *(.data*)
    } > PSRAM

    . = ALIGN(4);
    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } > PSRAM

    /* Discard everything else */
    /DISCARD/ : { *(.comment) *(.note*) *(.ARM*) }
}

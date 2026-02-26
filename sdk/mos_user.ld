/*
 * mos_user.ld - Linker script for ESP32-MOS user programs
 *
 * ESP32-S3 PSRAM is mapped into two virtual-address windows:
 *   IBUS (0x42000000): CPU fetches instructions from here
 *   DBUS (0x3C000000): CPU loads/stores data from here
 *
 * The loader writes the binary to s_exec_arena (DBUS=0x3C0E0000) and jumps
 * via the IBUS alias (0x420E0000). Since PMS is disabled in sdkconfig, the
 * CPU can also perform data loads from the IBUS range — so l32r reads of
 * literal pools at IBUS addresses (0x420E...) work correctly.
 *
 * Everything is linked at IBUS (MOS_EXEC_BASE = 0x420E0000):
 *   - code addresses → IBUS (CPU fetches fine)
 *   - string/rodata pointers in literal pool → IBUS (readable with PMS off)
 *
 * Entry: byte 0 = 'j _start' trampoline (mos_vector.S), jumps over literal pool
 * Prototype: int _start(int argc, char **argv, t_mos_api *mos);
 */

ENTRY(_start)

/* IBUS alias of s_exec_arena: dbus=0x3C0E0000, ibus=0x420E0000 */
MOS_EXEC_BASE = DEFINED(MOS_EXEC_BASE) ? MOS_EXEC_BASE : 0x420E0000;

MEMORY {
    /* PSRAM instruction-bus window on ESP32-S3 (0x42000000-0x44000000) */
    PSRAM (rwx) : ORIGIN = MOS_EXEC_BASE, LENGTH = 2M
}

SECTIONS {
    . = MOS_EXEC_BASE;

    /* Byte 0: jump trampoline — loader always jumps here.
     * KEEP so --gc-sections doesn't discard it. */
    .text.vector : {
        KEEP(*(.text.vector))
    } > PSRAM

    /* Literal pools — placed immediately after trampoline, before _start */
    .literal : {
        *(.literal .literal.* .text.entry.literal .text.*.literal)
    } > PSRAM

    .text : {
        *(.text.entry)   /* _start */
        *(.text*)
        *(.rodata*)
        *(.rodata)
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

    /DISCARD/ : { *(.comment) *(.note*) *(.ARM*) }
}

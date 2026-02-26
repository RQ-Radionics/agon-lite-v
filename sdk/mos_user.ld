/*
 * mos_user.ld - Linker script for ESP32-MOS user programs
 *
 * Produces a flat binary (objcopy -O binary) suitable for loading into
 * PSRAM by mos_loader.  The binary is position-DEPENDENT — it must be
 * loaded at exactly MOS_EXEC_BASE (default 0x3C000000).
 *
 * Entry point: _start
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

    .text : {
        *(.text.entry)   /* _start must be first */
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

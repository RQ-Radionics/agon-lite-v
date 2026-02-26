/* mos_vector.S - Entry trampoline for ESP32-MOS user programs
 *
 * The loader jumps to byte 0 of the binary (MOS_EXEC_BASE).
 * This 3-byte 'j _start' branches over the literal pool to _start.
 * Placed in .text.vector which the linker script puts at offset 0.
 */

    .section .text.vector, "ax"
    .global _mos_vector
    .type   _mos_vector, @function
    .align  4
_mos_vector:
    j   _start
    .size _mos_vector, .-_mos_vector

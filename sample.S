.thumb
.cpu cortex-m0
.syntax unified
.align 2
.org    0x00000000
.global _start

_sp: .word 0x20001000
_reset: .word 0x00000009

_start:
    movs r0, #64
    movs r1, #32
    b forward
back:
    movs r2, #16
    b fin

forward:
    b back

fin:
    movs r0, #0x18 // SYS_EXIT
    bkpt 0xAB      // Semihosting

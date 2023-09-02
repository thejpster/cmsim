.thumb
.cpu cortex-m0
.syntax unified
.align 2
.org    0x00000000
.global _start

.word 0x20001000
.word _start + 1

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
    b .

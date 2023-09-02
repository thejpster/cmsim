.global _start
.thumb
.word 0x20001000
.word 0x00000009

_start:
    mov r0, #64
    mov r1, #32
    b forward
back:
    mov r2, #16
    b fin

forward:
    b back

fin:
    b .

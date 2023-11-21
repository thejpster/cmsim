.syntax unified
.thumb
.cpu cortex-m0plus
.extern __sidata
.extern __sdata
.extern __edata
.global _start
.org    0x00000000

// Vector Table

.data
.section .vector_table.reset_vector
.word _start
.section .vector_table.exceptions
.word 0xEEEEEE00
.word 0xEEEEEE01
.word 0xEEEEEE02
.word 0xEEEEEE03
.word 0xEEEEEE04
.word 0xEEEEEE05

// Program Code

.text
.thumb_func
_start:
    // setup
    bl init
    // UART data address is 0x5930_3000
    ldr r4, =0x59303000
    // start of our message
    ldr r1, =message
    // Enable UART, putting 0x3 in UART CONTROL
    movs r2,3
    str r3,[r4,8]
    // byte counter
    movs r0, #0
loop:
    // get character, with r1 as base and r0 as index
    ldrb r2, [r1, r0]
    // if NUL, goto fin
    cmp r2, #0
    beq fin
    // write to uart
    str r2, [r4, 0]
    // move index up by 1 byte
    adds r0, r0, 1
    // go again
    b loop
fin:
    // SYS_EXIT
    movs r0, #0x18
    // Do Semihosting
    bkpt 0xAB
    b .

init:
    // Copy from __sidata to __sdata until we reach __edata
    ldr r0,=__sdata
    ldr r1,=__sidata
    ldr r2,=__edata
1:  cmp r0,r2
    beq 2f
    ldmia r1!,{r3}
    stmia r0!,{r3}
    b 1b
2:  // Write zeroes between __sbss and __ebss
    ldr r0,=__sbss
    ldr r1,=__ebss
    movs r2,0
3:  cmp r0,r1
    beq 4f
    stmia r0!,{r2}
    b 3b
4:  bx lr

// Mutable Global Variables

.data

some_global:
    .word 0xDEADBEEF

// Immutable Global Variables

.section .rodata

message:
    .ascii "Hello, I am the sample!\n\0"

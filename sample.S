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
.section .text
_start:
    // setup
    bl init
    // byte counter
    movs r0, #0
    // a literal zero
    movs r3, #0
    // UART data address is 0x5930_3000
    ldr r4, =uart_addr
    ldr r4,[r4]
    // start of our message
    ldr r1, =message
loop:
    // get character
    ldrb r2, [r1, r0]
    // is it zero
    cmp r2, r3
    // if zero goto fin
    beq fin
    // write to uart
    str r2, [r4, 0]
    // move index
    adds r0, r0, 1
    // go again
    b loop
fin:
    movs r0, #0x18 // SYS_EXIT
    bkpt 0xAB      // Semihosting

init:
    // Copy from __sidata to __sdata until we reach __edata
    ldr r0,=__sdata
    ldr r1,=__sidata
    ldr r2,=__edata
    movs r5,0
1:
    cmp r0,r2
    bne 2f
    b 3f
2:
    ldr r3,[r1]
    str r3,[r0]
    adds r0,r0,4
    adds r1,r1,4
    b 1b
3:
    // Write zeroes between __sbss and __ebss
    ldr r0,=__sbss
    ldr r1,=__ebss
    movs r2,0
4:
    cmp r0,r1
    bne 5f
    bx lr
5:
    str r2,[r0]
    adds r0,r0,4
    b 4b

// Global variables

.section .data
some_global:
    .word 0xDEADBEEF

// Constants

.section .rodata
.align 2
uart_addr:
    .word 0x59303000
.align 2
message:
    .ascii "Hello, I am the sample!\n\0"

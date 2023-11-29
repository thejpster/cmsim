# Linker script for sample.S
#
# Copyright (c) 2023 Jonathan 'theJPster' Pallant
# SPDX-License-Identifier: MIT OR Apache-2.0

EXTERN(_start);

MEMORY {
    FLASH : ORIGIN = 0x00000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS {
    PROVIDE(_ram_start = ORIGIN(RAM));
    PROVIDE(_ram_end = ORIGIN(RAM) + LENGTH(RAM));
    PROVIDE(_start_start = _ram_end);

    .vector_table ORIGIN(FLASH) :
    {
        __vector_table = .;

        LONG(_start_start & 0xFFFFFFF8);

        KEEP(*(.vector_table.reset_vector));
        __reset_vector = .;

        KEEP(*(.vector_table.exceptions));
        __eexceptions = .;

        KEEP(*(.vector_table.interrupts));
    } > FLASH

    PROVIDE(_stext = ADDR(.vector_table) + SIZEOF(.vector_table));

    .text _stext :
    {
        __stext = .;
        *(.Reset);
        *(.text .text.*);
        *(.HardFault.*);

        . = ALIGN(4);

        __etext = .;
    } > FLASH

    .rodata : ALIGN(4)
    {
        . = ALIGN(4);
        __srodata = .;
        *(.rodata .rodata.*);
        . = ALIGN(4);
        __erodata = .;   
    } > FLASH

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        __sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        __edata = .;   
    } > RAM AT>FLASH

    __sidata = LOADADDR(.data);

    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __sbss = .;
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
    } > RAM
    . = ALIGN(4);
    __ebss = .;

    .uninit (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __suninit = .;
        *(.uninit .uninit.*);
        . = ALIGN(4);
        __euninit = .;
    } > RAM

    PROVIDE(__sheap = __euninit);
}

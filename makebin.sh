#!/bin/bash

set -euo pipefail

mkdir -p ./target
arm-none-eabi-as sample.S -o ./target/sample.o
arm-none-eabi-ld ./target/sample.o -Tsample.ld -o ./target/sample.elf
arm-none-eabi-objdump -d ./target/sample.elf
readelf -x .rodata ./target/sample.elf
readelf -x .bss ./target/sample.elf
readelf -x .data ./target/sample.elf

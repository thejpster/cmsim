#!/bin/sh

mkdir -p ./target
arm-none-eabi-as sample.S -o ./target/sample.elf
arm-none-eabi-objcopy -O binary ./target/sample.elf ./target/sample.bin
arm-none-eabi-objdump -d ./target/sample.elf
hexdump -C ./target/sample.bin

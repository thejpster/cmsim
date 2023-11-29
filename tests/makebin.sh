#!/usr/bin/env bash

# Copyright (c) 2023 Jonathan 'theJPster' Pallant
# SPDX-License-Identifier: MIT OR Apache-2.0

set -euo pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

OUTDIR=${1:-./output}

mkdir -p ${OUTDIR}
arm-none-eabi-as ${SCRIPT_DIR}/sample.S -o ${OUTDIR}/sample.o
arm-none-eabi-ld ${OUTDIR}/sample.o -T ${SCRIPT_DIR}/sample.ld -o ${OUTDIR}/sample.elf

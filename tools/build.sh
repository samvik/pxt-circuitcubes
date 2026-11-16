#!/bin/bash
# Full local build: compiles main.ts + test.ts + circuitcubes.cpp for micro:bit v2.
# Output: built/mbcodal-binary.hex  (copy it to the MICROBIT drive)
#
# pxt's CODAL step runs in the Docker image pext/yotta:latest, which current Docker
# can't pull. tools/shim/docker intercepts that one call and runs CODAL's build.py
# on the host instead (needs arm-none-eabi-gcc, cmake, python3, git).
# Prereqs: `npm i -g pxt`, then in this folder `pxt target microbit && pxt install`.
cd "$(dirname "$0")/.."
PATH="$PWD/tools/shim:$PATH" PXT_FORCE_LOCAL=1 pxt build && ls -la built/mbcodal-binary.hex

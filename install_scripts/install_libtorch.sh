#!/usr/bin/env bash

set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd ${CURRENT_DIR}
cd ..
cd third_party

sudo apt-get install unzip

TARGET_ARCH=x86_64


wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcpu.zip

unzip libtorch-cxx11-abi-shared-with-deps-1.10.0+cpu.zip.1

mv libtorch ./install/libtorch_cpu

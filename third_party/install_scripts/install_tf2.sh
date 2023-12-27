#!/usr/bin/env bash


# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
#. ./installer_base.sh

cmake ..      -DCMAKE_INSTALL_PREFIX=../../install/tf2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTS=OFF

make -j6

sudo make install
    
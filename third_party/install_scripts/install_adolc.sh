#!/usr/bin/env bash


# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# Related projects
# https://github.com/CSCsw/ColPack.git
# https://github.com/coin-or/ADOL-C

sudo apt-get -y install libcolpack-dev libadolc-dev
    
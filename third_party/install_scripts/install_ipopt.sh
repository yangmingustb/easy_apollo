#!/usr/bin/env bash


# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
#. ./installer_base.sh

sudo apt-get install --no-install-recommends coinor-libipopt-dev

#FIXME(all): dirty hack here.
sudo sed -i '/#define __IPSMARTPTR_HPP__/a\#define HAVE_CSTDDEF' \
    /usr/include/coin/IpSmartPtr.hpp

# Source Code Package Link: https://github.com/coin-or/Ipopt/releases
    
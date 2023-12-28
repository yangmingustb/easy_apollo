#!/usr/bin/env bash

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/ad_rss_lib"


cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch v1.1.0 git@github.com:intel/ad-rss-lib.git

cd ad-rss-lib

mkdir build && cd build

cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=./../../install/ad_rss_lib -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF 

make -j6
make install

cd ..
cp -r ./src/situation ./../install/ad_rss_lib/include/ad_rss


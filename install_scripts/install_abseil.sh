
set -e

cd /tmp
rm -rf abseil

git clone --depth 1 git@github.com:abseil/abseil-cpp.git

cd abseil-cpp
mkdir build
cd build

cmake .. 
make -j
sudo make install
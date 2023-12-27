set -e

cd /tmp
rm -rf googletest
git clone --depth 1 --branch release-1.11.0 git@github.com:google/googletest.git
cd googletest && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
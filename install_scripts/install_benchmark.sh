

set -e

cd /tmp
rm -rf benchmark
# git clone --depth 1 --branch v1.5.0 git@github.com:google/benchmark.git
git clone --depth 1 git@github.com:google/benchmark.git
cd benchmark 

# git clone --branch release-1.8.1 --depth 1 git@github.com:google/googletest.git
git clone --depth 1 git@github.com:google/googletest.git
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j$(nproc)
sudo make install
sudo ldconfig
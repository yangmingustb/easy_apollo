set -e



CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/googletest"


cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch release-1.10.0 git@github.com:google/googletest.git

cd googletest


mkdir -p build && cd build
CXXFLAGS="-fPIC" cmake -DCMAKE_CXX_FLAGS="-w" -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install

sudo ldconfig
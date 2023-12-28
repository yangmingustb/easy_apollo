# sudo apt-get install libgoogle-glog-dev

set -e



CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/glog"


cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch v0.5.0 git@github.com:google/glog.git

cd glog


mkdir -p build && cd build

CXXFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DBUILD_SHARED_LIBS=ON ..

make -j$(nproc)
sudo make install

sudo ldconfig
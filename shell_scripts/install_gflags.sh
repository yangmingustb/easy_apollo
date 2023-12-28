set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/gflags"


cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch v2.2.0 git@github.com:gflags/gflags.git

cd gflags && mkdir build && cd build

CXXFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DBUILD_SHARED_LIBS=ON ..

make
sudo make install

sudo ldconfig


# pip install python-gflags==3.1.2

set -e

CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PREFIX="$CURRENT_PATH/../third_party/install/abseil-cpp"


cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch 20200225.2 git@github.com:abseil/abseil-cpp.git

cd abseil-cpp
mkdir build
cd build

cmake ..       -DBUILD_SHARED_LIBS=ON    -DCMAKE_INSTALL_PREFIX=./../../install/abseil-cpp -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17

make -j6
sudo make install
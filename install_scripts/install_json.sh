
set -e



CURRENT_PATH=$(cd $(dirname $0) && pwd)



cd $CURRENT_PATH
cd ..
cd third_party

git clone --depth 1 --branch v3.8.0 git@github.com:nlohmann/json.git

cd json


mkdir -p build && cd build


cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=../../install/json -DCMAKE_BUILD_TYPE=Release

make -j6
make install

sudo ldconfig

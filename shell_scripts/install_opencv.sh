
# 二进制安装
# sudo apt-get install libcv-dev
# sudo apt-get install libopencv-dev


# another way:源码安装，默认的程序无法编译，需要修改一些值
set -e

# sudo apt install libvtk7-dev 

# cd /tmp
# rm -rf opencv
# git clone --depth 1 --branch 4.2.0 git@github.com:opencv/opencv.git
# cd opencv && mkdir build && cd build
# cmake .. -DCMAKE_INSTALL_PREFIX=~/jimu-auto/third_party/install/opencv -DCMAKE_BUILD_TYPE=Release

# cmake .. -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules/ -DCMAKE_INSTALL_PREFIX=~/jimu-auto/third_party/install/opencv -DCMAKE_BUILD_TYPE=Release


sudo apt-get install -y libgtk2.0-dev


CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd ${CURRENT_DIR}
cd ..
cd third_party
cd opencv
rm -rf build
mkdir build && cd build

cmake .. -DCMAKE_INSTALL_PREFIX=./../../install/opencv -DCMAKE_BUILD_TYPE=Release

make -j6
make install


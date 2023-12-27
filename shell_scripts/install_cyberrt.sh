
set -e


cd ~/nop_baidu/third_party
# rm -rf CyberRT

git clone --depth 1 --recursive --branch v7.0.0 git@github.com:minhanghuang/CyberRT.git
 

# cd CyberRT
# mkdir build
# cd build
# cmake .. -DBUILD_SHARED_LIBS=ON

# make -j8

# sudo make install

# sudo ldconfig

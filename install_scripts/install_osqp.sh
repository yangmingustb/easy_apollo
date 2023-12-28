
set -e


cd /tmp
rm -rf osqp

git clone --depth 1 --recursive --branch v0.5.0 git@gitee.com:diracat/osqp.git
 

cd osqp
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON

make -j8

sudo make install

sudo ldconfig

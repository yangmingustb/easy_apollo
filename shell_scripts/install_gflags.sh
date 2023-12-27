set -e

cd /tmp
rm -rf gflags

git clone --depth 1 git@github.com:gflags/gflags.git

cd gflags && mkdir build && cd build

cmake ..  -DBUILD_SHARED_LIBS=ON
make
sudo make install

sudo ldconfig


# pip install python-gflags==3.1.2
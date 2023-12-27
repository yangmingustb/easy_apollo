
set -e


# https://github.com/civetweb/civetweb/archive/v1.11.tar.gz


# https://cloud.tencent.com/developer/article/1913473

# need two minutes

cd /tmp
rm -rf civetweb
git clone --depth 1 https://github.com/civetweb/civetweb.git
cd civetweb


mkdir buildx && cd buildx
cmake -DCIVETWEB_ENABLE_CXX=ON -DBUILD_SHARED_LIBS=ON -DCIVETWEB_ENABLE_WEBSOCKETS=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig


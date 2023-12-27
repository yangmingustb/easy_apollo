set -e

# apt install:
# sudo apt-get install openscenegraph

cd /tmp
rm -rf OpenSceneGraph
git clone --depth 1 --branch OpenSceneGraph-3.6.5 https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph && mkdir build && cd build
cmake -DOSG_TEXT_USE_FONTCONFIG=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig
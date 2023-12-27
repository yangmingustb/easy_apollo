cd ..
cd PROJ

rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../install/proj4 -DCMAKE_BUILD_TYPE=Release
        # -DBUILD_SHARED_LIBS=ON 
        #-DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \

make -j8
sudo make install
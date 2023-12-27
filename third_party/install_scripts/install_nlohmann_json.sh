#!/usr/bin/env bash


# Fail on first error.
#后续所有的bash命令的返回code如果不是0，那么脚本立即退出
set -e

#cd "$(dirname "${BASH_SOURCE[0]}")"
#使用 . 号引用文件, 也可以替换为 source 
. ./install_base.sh

PKG_BASE_NAME=nlohmann_json
PKG_VER="3.8.0"
PKG_NAME="${PKG_BASE_NAME}-${PKG_VER}"
ZIP_NAME=${PKG_NAME}".tar.gz"


CHECKSUM="7d0edf65f2ac7390af5e5a0b323b31202a6c11d744a74b588dc30f5a8c9865ba"

DOWNLOAD_LINK="https://github.com/nlohmann/json/archive/v${PKG_VER}.tar.gz"
download_if_not_cached "${ZIP_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

#tar xzf "${PKG_NAME}"

#删除目录
delete_dir "./${PKG_NAME}"
#将文件解压到指定目录
mkdir ./${PKG_NAME} && tar -xzvf ${ZIP_NAME} -C ./${PKG_NAME} --strip-components 1

#切换目录
pushd "${PKG_NAME}"
    #删除目录
    delete_dir "./build"

    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
	    -DCMAKE_INSTALL_PREFIX=../../install/${PKG_BASE_NAME} \
        -DCMAKE_BUILD_TYPE=Release
        #-DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
    #make -j$(nproc)
    make -j4
    make install
popd

#rm -rf "${PKG_NAME}" "${ZIP_NAME}"

#ldconfig

ok "Successfully installed ${PKG_NAME}"

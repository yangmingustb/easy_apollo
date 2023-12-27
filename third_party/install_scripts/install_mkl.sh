#!/usr/bin/env bash



set -e
. ./install_base.sh

TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" != "x86_64" ]]; then
    exit 0
fi

pip3_install mkl==2021.1.1

# Workaround for removing duplicate entries in mkl PYPI installation
function mkl_relink {
    MKL_LIBDIR="/usr/local/lib"
    for so in ${MKL_LIBDIR}/libmkl*.so; do
        so1="${so}.1"

        if [[ "$(basename ${so})" == "libmkl_sycl.so" ]]; then
            rm -f ${so} ${so1} || true
            continue
        fi

        if [[ -f "${so}.1" ]]; then
            cs1=$(sha256sum ${so1} | awk '{print $1}')
            cs0=$(sha256sum ${so} | awk '{print $1}')
            if [[ "${cs1}" == "${cs0}" ]]; then
                so1_name="$(basename $so1)"
                warning "Duplicate so ${so} with ${so1_name} found, re-symlinking..."
                info "Now perform: rm -f ${so} && ln -s ${so1_name} ${so}"
                rm -f ${so} && ln -s ${so1_name} ${so}
            fi
        fi
    done
}

function tbb_relink() {
    TBB_LIBDIR="/usr/local/lib"
    for so in ${TBB_LIBDIR}/libtbb*.so*; do
        soname="$(basename ${so})"
        IFS='.' read -ra arr <<< "${soname}"
        IFS=' ' # restore IFS
        num=${#arr[@]}
        if [[ ${num} != 4 ]]; then # Keep only libtbb.so.12.1
            rm -f ${so} || true
        fi
    done

    for so in ${TBB_LIBDIR}/libtbb*.so*; do
        soname="$(basename ${so})"
        IFS='.' read -ra arr <<< "${soname}"
        IFS=' ' # restore IFS
        core="${arr[0]}.so"
        sudo ln -s ${soname} ${TBB_LIBDIR}/${core}.${arr[2]}
        sudo ln -s ${core}.${arr[2]} ${TBB_LIBDIR}/${core}
    done
}

mkl_relink
tbb_relink
sudo ldconfig

ok "Successfully installed mkl from PYPI"
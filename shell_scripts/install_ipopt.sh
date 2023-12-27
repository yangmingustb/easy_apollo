# Fail on first error.
set -e




# 1, 直接使用库安装
# https://gitee.com/YaoDecheng/ipopt_install?_from=gitee_search#%E4%B8%89%E5%91%BD%E4%BB%A4%E8%A1%8C%E5%AE%89%E8%A3%85

#ubuntu 1804
sudo apt-get install -y coinor-libipopt-dev

#据博客作者反映，源码安装后期使用容易出错



cd "$(dirname "${BASH_SOURCE[0]}")"
#. ./installer_base.sh

#FIXME(all): dirty hack here.
sudo sed -i '/#define __IPSMARTPTR_HPP__/a\#define HAVE_CSTDDEF' \
    /usr/include/coin/IpSmartPtr.hpp



# 2,源码安装
# sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev

# git clone git@gitee.com:lebment/Ipopt.git
# cd Ipopt
# ./configure
# # 一定要看到配置成功
 
# # 编译
# make
# make test
# make install







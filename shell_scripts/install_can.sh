

# 去官网下载最新版本即可
# https://www.kvaser.cn/downloads/#?filter=linuxca
#搜索linux can

#参考1
# https://blog.csdn.net/aspirationmars/article/details/115671853?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-1-115671853-blog-125757630.235^v38^pc_relevant_sort_base2&spm=1001.2101.3001.4242.2&utm_relevant_index=4

#参考2
#https://blog.csdn.net/qq_43569735/article/details/108037140?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-108037140-blog-125757630.235%5Ev38%5Epc_relevant_sort_base2&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-108037140-blog-125757630.235%5Ev38%5Epc_relevant_sort_base2&utm_relevant_index=5

set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd ${CURRENT_DIR}
cd ..
cd third_party
cd linuxcan

make -j6
sudo make install
# sudo make uninstall

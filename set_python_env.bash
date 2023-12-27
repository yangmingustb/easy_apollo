#!/bin/bash


SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PWD=$(pwd)
echo $PWD

PYTHONPATH+=":$PWD"
# PYTHONPATH+=":$PWD/src/cyber/proto"
PYTHONPATH+=":$PWD/build"
# PYTHONPATH+=":$PWD/build/lib/cyber/python/internal"
# PYTHONPATH+=":$PWD/build/lib"

echo $PYTHONPATH
export PYTHONPATH

# export PYTHONPATH=./build:./src

# LD_LIBRARY_PATH+=":$PWD/lib"
# export LD_LIBRARY_PATH=":$PWD/build/lib"

LD_LIBRARY_PATH+=":$PWD"
LD_LIBRARY_PATH+=":$PWD/third_party/install/abseil-cpp/lib"
LD_LIBRARY_PATH+=":$PWD/third_party/install/ad_rss_lib/lib"



LD_LIBRARY_PATH+=":$PWD/third_party/install/osqp/lib"
LD_LIBRARY_PATH+=":$PWD/third_party/install/libtorch_cpu/lib"

LD_LIBRARY_PATH+=":$PWD/third_party/install/proj4/lib"
LD_LIBRARY_PATH+=":$PWD/third_party/install/opencv/lib"
LD_LIBRARY_PATH+=":$PWD/third_party/install/horizon_ai/lib"
LD_LIBRARY_PATH+=":$PWD/install/lib"
LD_LIBRARY_PATH+=":$PWD/build/lib"

echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH


export CYBER_PATH=$PWD/cyber
export sysmo_start=0
export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1


# export CYBER_TIME=`date +"%Y%m%d_%H%M%S"`
export CYBER_TIME=`date +"%Y%m%d_%H"`
echo $CYBER_TIME

export CYBER_LOG_PATH="./data/log"
cd ${PWD}  

echo $PWD

cd ${CYBER_LOG_PATH}  

mkdir $CYBER_TIME

cd ${SCRIPTS_PATH}


export GLOG_log_dir=$PWD/data/log/$CYBER_TIME

echo $GLOG_log_dir

export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

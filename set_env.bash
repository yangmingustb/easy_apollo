#!/bin/bash

PWD=$(pwd)
echo $PWD

LD_LIBRARY_PATH+=":$PWD"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/abseil-cpp/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/ad_rss_lib/lib"



LD_LIBRARY_PATH+=":$PWD/../third_party/install/osqp/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/libtorch_cpu/lib"

LD_LIBRARY_PATH+=":$PWD/../third_party/install/proj4/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/opencv/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/googletest/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/glog/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/gflags/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/fastrtps/lib"

LD_LIBRARY_PATH+=":$PWD/lib"

echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH

echo $PWD


SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)


export CYBER_PATH=${SCRIPTS_PATH}/../cyber

echo $CYBER_PATH

# export CYBER_DOMAIN_ID=80
# export CYBER_IP=127.0.0.1

# export CYBER_TIME=`date +"%Y%m%d_%H%M%S"`
export CYBER_TIME=`date +"%Y%m%d_%H"`
echo $CYBER_TIME


export CYBER_LOG_PATH="./../data/log"
cd ${SCRIPTS_PATH}  

echo $SCRIPTS_PATH

cd ${CYBER_LOG_PATH}  

mkdir $CYBER_TIME

cd ${SCRIPTS_PATH}


export GLOG_log_dir=${SCRIPTS_PATH}/../data/log/${CYBER_TIME}
# export GLOG_log_dir=${SCRIPTS_PATH}/../data/log

echo $GLOG_log_dir

export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

# export cyber_trans_perf=0
# export cyber_sched_perf=0

# export sysmo_start=0


# for DEBUG log
#export GLOG_minloglevel=-1
#export GLOG_v=4







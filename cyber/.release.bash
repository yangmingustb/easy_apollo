#! /usr/bin/env bash
TOP_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
source ${TOP_DIR}/scripts/apollo.bashrc

export CYBER_PATH="${APOLLO_ROOT_DIR}/cyber"

pathprepend "${TOP_DIR}/bin"

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

export CYBER_TIME=`date +"%Y%m%d_%H%M%S"`
echo $CYBER_TIME


export GLOG_log_dir=${APOLLO_ROOT_DIR}/data/log/$CYBER_TIME

echo $GLOG_log_dir

export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

# for DEBUG log
#export GLOG_v=4

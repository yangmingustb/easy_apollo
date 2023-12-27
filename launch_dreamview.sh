#!/bin/bash
#

source ./set_env.bash
source ./setup.bash


DREAMVIEW_URL="http://localhost:8888"

start_procs="dreamview_main"


SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)


export COMMON_PATH=${SCRIPTS_PATH}/../modules/common

for proc in $start_procs; do
    if [ "$proc"x = "dreamview_main"x ]; then
            ./$proc --flagfile=${COMMON_PATH}/data/global_flagfile.txt &      
        echo $proc
    else 
        ./$proc &
    fi
done

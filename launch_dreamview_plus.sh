#!/bin/bash
#

source ./set_env.bash
source ./setup.bash


DREAMVIEW_URL="http://localhost:8888"

start_procs="dreamview2_main"


SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)


export DV2_PATH=${SCRIPTS_PATH}/../modules/dreamview_plus

for proc in $start_procs; do
    if [ "$proc"x = "dreamview2_main"x ]; then
            ./$proc --flagfile=${DV2_PATH}/conf/dreamview.conf &      
        echo $proc
    else 
        ./$proc &
    fi
done

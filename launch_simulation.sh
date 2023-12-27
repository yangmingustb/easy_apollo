#!/bin/bash
#

PWD=$(pwd)
echo $PWD

source ./set_env.bash
source ./setup.bash



start_procs="viz2d_component_main control_component_main virtual_chassis_component_main planning_component_main"


SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)


export PLANNING_PATH=${SCRIPTS_PATH}/../modules/planning
export CONTROL_PATH=${SCRIPTS_PATH}/../modules/control
export COMMON_PATH=${SCRIPTS_PATH}/../modules/common

for proc in $start_procs; do
    #save $start_procs output to log, make terminal clean
    if [ "$proc"x = "planning_component_main"x ]; then
        # GLOG_v=4 ./$proc --flagfile=${PLANNING_PATH}/conf/planning.conf &
        ./$proc --flagfile=${PLANNING_PATH}/conf/planning.conf &
        echo $proc
    elif [ "$proc"x = "control_component_main"x ]; then
            ./$proc --flagfile=${CONTROL_PATH}/conf/control.conf &
        echo $proc
    elif [ "$proc"x = "viz2d_component_main"x ]; then
            ./$proc --flagfile=${COMMON_PATH}/data/global_flagfile.txt &      
        echo $proc
    elif [ "$proc"x = "virtual_chassis_component_main"x ]; then
            ./$proc --flagfile=${COMMON_PATH}/data/global_flagfile.txt &      
        echo $proc
    else 
        ./$proc &
    fi
done

#!/bin/bash
#

working_dir="./"
ended_procs="
            viz2d_component_main
            viz3d_component_main
            control_component_main
            virtual_chassis_component_main
            planning_component_main
            chassis_component_main
            cyber_recorder
            dreamview_main
            "

curr_dir=$(pwd)
cd $working_dir

for proc in $ended_procs; do
    echo "kill $proc"
    proc_trunc=${proc:0:14}
    ps_proc=`ps -A | grep ${proc_trunc} | grep -v grep`
    if [ "$ps_proc"x != ""x ]; then
        $(killall $proc > /dev/null 2>&1 )
        sleep 1
    fi
done



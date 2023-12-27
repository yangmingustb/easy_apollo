#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import sys
import threading

import gflags
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from cyber.python.cyber_py3 import cyber
from modules.control.proto import control_cmd_pb2
from modules.planning.proto import planning_pb2


LAST_TRAJ_DATA = []
LAST_TRAJ_T_DATA = []
CURRENT_TRAJ_DATA = []
CURRENT_TRAJ_T_DATA = []
INIT_V_DATA = []
INIT_T_DATA = []

st_t = []
st_s = []
st_v = []
st_acc = []
st_jerk = []

# st_bounds_decider
st_bounds_decider_obs_t=[]
st_bounds_decider_obs_s=[]

st_bounds_decider_drive_region_t=[]
st_bounds_decider_drive_region_s=[]

st_bounds_decider_guide_line_t=[]
st_bounds_decider_guide_line_s=[]

# speed_bounds_pri_decider
speed_decider_obs_t=[]
speed_decider_obs_s=[]

speed_decider_t_range=[]
speed_decider_s_range=[]


speed_limit_s=[]
speed_limit_v=[]

# speed_bounds_final_decider
speed_decider_final_obs_t=[]
speed_decider_final_obs_s=[]

# dp
dp_curve_t=[]
dp_curve_s=[]
dp_curve_v=[]
dp_curve_acc=[]
dp_curve_jerk=[]

# qp
qp_curve_t=[]
qp_curve_s=[]
qp_curve_v=[]
qp_curve_acc=[]
qp_curve_jerk=[]

# nlp
nlp_curve_t=[]
nlp_curve_s=[]
nlp_curve_v=[]
nlp_curve_acc=[]
nlp_curve_jerk=[]

pwj_speed_drive_region_t=[]
pwj_speed_drive_region_s=[]

begin_t = None
last_t = None


nlp_speed_limit_s =[]
nlp_speed_limit_v =[]

lock = threading.Lock()

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Planning plot data length")

plot_st_decider=0

def callback(planning_pb):
    global INIT_V_DATA, INIT_T_DATA
    global CURRENT_TRAJ_DATA, LAST_TRAJ_DATA
    global CURRENT_TRAJ_T_DATA, LAST_TRAJ_T_DATA
    global begin_t, last_t
    global st_t,st_s,st_v,st_acc,st_jerk
    global speed_limit_s
    global speed_limit_v
    global st_bounds_decider_obs_t
    global st_bounds_decider_obs_s
    global st_bounds_decider_drive_region_t
    global st_bounds_decider_drive_region_s
    global st_bounds_decider_guide_line_t
    global st_bounds_decider_guide_line_s

    global dp_curve_t
    global dp_curve_s
    global dp_curve_v
    global dp_curve_acc
    global dp_curve_jerk

    global qp_curve_t
    global qp_curve_s   
    global qp_curve_v   
    global qp_curve_acc   
    global qp_curve_jerk   
    
    global nlp_curve_t   
    global nlp_curve_s   
    global nlp_curve_v   
    global nlp_curve_acc   
    global nlp_curve_jerk   
    
    global speed_decider_obs_t   
    global speed_decider_obs_s   
    global speed_decider_final_obs_t   
    global speed_decider_final_obs_s   

    global nlp_speed_limit_s   
    global nlp_speed_limit_v 

    global speed_decider_t_range 
    global speed_decider_s_range 

    global pwj_speed_drive_region_t 
    global pwj_speed_drive_region_s 



    lock.acquire()

    if begin_t is None:
        begin_t = planning_pb.header.timestamp_sec
    current_t = planning_pb.header.timestamp_sec
    if last_t is not None and abs(current_t - last_t) > 1:
        begin_t = planning_pb.header.timestamp_sec
        LAST_TRAJ_DATA = []
        LAST_TRAJ_T_DATA = []

        CURRENT_TRAJ_DATA = []
        CURRENT_TRAJ_T_DATA = []

        INIT_V_DATA = []
        INIT_T_DATA = []

    INIT_T_DATA.append(current_t - begin_t)
    INIT_V_DATA.append(planning_pb.debug.planning_data.init_point.v)

    LAST_TRAJ_DATA = []
    for v in CURRENT_TRAJ_DATA:
        LAST_TRAJ_DATA.append(v)

    LAST_TRAJ_T_DATA = []
    for t in CURRENT_TRAJ_T_DATA:
        LAST_TRAJ_T_DATA.append(t)

    CURRENT_TRAJ_DATA = []
    CURRENT_TRAJ_T_DATA = []


    st_t = []
    st_s = []
    st_v = []
    st_acc = []
    st_jerk = []

    for traj_point in planning_pb.trajectory_point:
        CURRENT_TRAJ_DATA.append(traj_point.v)
        CURRENT_TRAJ_T_DATA.append(current_t - begin_t + traj_point.relative_time)

        st_t.append(traj_point.relative_time)
        st_s.append(traj_point.path_point.s)

        st_v.append(traj_point.v)
        st_acc.append(traj_point.a)
        st_jerk.append(traj_point.da)



    
    speed_limit_s =[]
    speed_limit_v =[]
    st_bounds_decider_obs_t=[]
    st_bounds_decider_obs_s=[]

    speed_decider_final_obs_t=[]
    speed_decider_final_obs_s=[]

    st_bounds_decider_drive_region_t=[]
    st_bounds_decider_drive_region_s=[]
    st_bounds_decider_guide_line_t=[]
    st_bounds_decider_guide_line_s=[]

    speed_decider_obs_t=[]
    speed_decider_obs_s=[]

    nlp_speed_limit_s=[]
    nlp_speed_limit_v=[]
    
    pwj_speed_drive_region_t=[]
    pwj_speed_drive_region_s=[]






    for st_graph in planning_pb.debug.planning_data.st_graph:
        
        
        print(st_graph.name)
        print("traj number %d" %planning_pb.header.sequence_num)
        
        if(plot_st_decider):
            if 'ST_BOUNDS_DECIDER' == st_graph.name:
                # print(st_graph.name)
                for boundary in st_graph.boundary:
                    # print(1)
                    # print(boundary.type)
                    if boundary.type == 7:
                        for point in boundary.point:
                            st_bounds_decider_drive_region_t.append(point.t)
                            st_bounds_decider_drive_region_s.append(point.s)
                    else:
                        for point in boundary.point:
                            st_bounds_decider_obs_t.append(point.t)
                            st_bounds_decider_obs_s.append(point.s)

                for point in st_graph.st_guide_line:
                    st_bounds_decider_guide_line_t.append(point.t)
                    st_bounds_decider_guide_line_s.append(point.s)
        
        if 'SPEED_BOUNDS_PRIORI_DECIDER' == st_graph.name:
            for boundary in st_graph.boundary:
                # print(1)
                # print(boundary.type)
                for point in boundary.point:
                    speed_decider_obs_t.append(point.t)
                    speed_decider_obs_s.append(point.s)

            for point in st_graph.speed_limit:
                speed_limit_s.append(point.s)
                speed_limit_v.append(point.v)
                # print("s %f, v %f " %(point.s, point.v))

        if 'SPEED_BOUNDS_FINAL_DECIDER' == st_graph.name:
            for boundary in st_graph.boundary:
                # print(1)
                # print(boundary.type)
                for point in boundary.point:
                    speed_decider_final_obs_t.append(point.t)
                    speed_decider_final_obs_s.append(point.s)

        if 'PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER' == st_graph.name:
            # print(st_graph.name)
            for point in st_graph.speed_limit:
                nlp_speed_limit_s.append(point.s)
                nlp_speed_limit_v.append(point.v)
            
            for boundary in st_graph.boundary:
                    # print(1)
                    # print(boundary.type)
                    if boundary.type == 7:
                        for point in boundary.point:
                            pwj_speed_drive_region_t.append(point.t)
                            pwj_speed_drive_region_s.append(point.s)

                            print("t %f, s %f " %(point.t, point.s))


    qp_curve_t=[]
    qp_curve_s=[]
    qp_curve_v=[]
    qp_curve_acc=[]
    qp_curve_jerk=[]

    dp_curve_t=[]   
    dp_curve_s=[]
    dp_curve_v=[]
    dp_curve_acc=[]
    dp_curve_jerk=[]

    nlp_curve_t=[]
    nlp_curve_s=[]
    nlp_curve_v=[]
    nlp_curve_acc=[]
    nlp_curve_jerk=[]

    for st in planning_pb.debug.planning_data.speed_plan:
        
        # print(st.name)
        if(st.name == 'SPEED_HEURISTIC_OPTIMIZER'):
            for point in st.speed_point:
                dp_curve_t.append(point.t)
                dp_curve_s.append(point.s)
                dp_curve_v.append(point.v)
                dp_curve_acc.append(point.a)
                dp_curve_jerk.append(point.da)
        
        if(st.name == 'PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER'):
            for point in st.speed_point:
                nlp_curve_t.append(point.t)
                nlp_curve_s.append(point.s)
                nlp_curve_v.append(point.v)
                nlp_curve_acc.append(point.a)
                nlp_curve_jerk.append(point.da)
        
        if(st.name == 'qp_speed'):
            for point in st.speed_point:
                qp_curve_t.append(point.t)
                qp_curve_s.append(point.s)
                qp_curve_v.append(point.v)
                qp_curve_acc.append(point.a)
                qp_curve_jerk.append(point.da)


    print("====================")
    speed_decider_t_range=[]
    speed_decider_s_range=[]

    speed_decider_t_range.append(0)
    if(len(dp_curve_t) > 1):
    
        speed_decider_t_range.append(dp_curve_t[-1])
    else:
        speed_decider_t_range.append(7)

    max_s=0.0   
    for path in planning_pb.debug.planning_data.path:
        print(path.name)

        if('PlanningPathData' == path.name):
            for point in path.path_point:
                max_s = max(max_s,point.s )

    speed_decider_s_range.append(max_s)
    speed_decider_s_range.append(max_s)
    print(speed_decider_s_range)

    lock.release()

    last_t = current_t




def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lock.acquire()

    st_curve.set_xdata(st_t)
    st_curve.set_ydata(st_s)

    speed_decider_s_range_curve.set_xdata(speed_decider_t_range)
    speed_decider_s_range_curve.set_ydata(speed_decider_s_range)

    if(plot_st_decider):
        st_decider_obs_curve.set_xdata(st_bounds_decider_obs_t)
        st_decider_obs_curve.set_ydata(st_bounds_decider_obs_s)

    speed_decider_obs_curve.set_xdata(speed_decider_obs_t)
    speed_decider_obs_curve.set_ydata(speed_decider_obs_s)

    speed_decider_final_obs_curve.set_xdata(speed_decider_final_obs_t)
    speed_decider_final_obs_curve.set_ydata(speed_decider_final_obs_s)

    if(plot_st_decider):
        st_decider_drive_bound.set_xdata(st_bounds_decider_drive_region_t)
        st_decider_drive_bound.set_ydata(st_bounds_decider_drive_region_s)

        st_decider_guide_line.set_xdata(st_bounds_decider_guide_line_t)
        st_decider_guide_line.set_ydata(st_bounds_decider_guide_line_s)

    # dp
    dp_curve.set_xdata(dp_curve_t)
    dp_curve.set_ydata(dp_curve_s)

    dp_tv_curve.set_xdata(dp_curve_t)
    dp_tv_curve.set_ydata(dp_curve_v)

    dp_ta_curve.set_xdata(dp_curve_t)
    dp_ta_curve.set_ydata(dp_curve_acc)

    dp_tjerk_curve.set_xdata(dp_curve_t)
    dp_tjerk_curve.set_ydata(dp_curve_jerk)

    #qp
    qp_curve.set_xdata(qp_curve_t)
    qp_curve.set_ydata(qp_curve_s)

    qp_tv_curve.set_xdata(qp_curve_t)
    qp_tv_curve.set_ydata(qp_curve_v)

    qp_ta_curve.set_xdata(qp_curve_t)
    qp_ta_curve.set_ydata(qp_curve_acc)

    qp_tjerk_curve.set_xdata(qp_curve_t)
    qp_tjerk_curve.set_ydata(qp_curve_jerk)

    # nlp
    nlp_curve.set_xdata(nlp_curve_t)
    nlp_curve.set_ydata(nlp_curve_s)

    nlp_tv_curve.set_xdata(nlp_curve_t)
    nlp_tv_curve.set_ydata(nlp_curve_v)

    nlp_ta_curve.set_xdata(nlp_curve_t)
    nlp_ta_curve.set_ydata(nlp_curve_acc)

    nlp_tjerk_curve.set_xdata(nlp_curve_t)
    nlp_tjerk_curve.set_ydata(nlp_curve_jerk)

    pwj_speed_drive_bound.set_xdata(pwj_speed_drive_region_t)
    pwj_speed_drive_bound.set_ydata(pwj_speed_drive_region_s)

    tv_curve.set_xdata(st_t)
    tv_curve.set_ydata(st_v)
    
    sa_curve.set_xdata(st_t)
    sa_curve.set_ydata(st_acc)
    
    sj_curve.set_xdata(st_t)
    sj_curve.set_ydata(st_jerk)

    speed_limit_curve.set_xdata(speed_limit_s)
    speed_limit_curve.set_ydata(speed_limit_v)

    nlp_speed_limit_curve.set_xdata(nlp_speed_limit_s)
    nlp_speed_limit_curve.set_ydata(nlp_speed_limit_v)

    sv_curve.set_xdata(st_s)
    sv_curve.set_ydata(st_v)

    lock.release()
    #brake_text.set_text('brake = %.1f' % brake_data[-1])
    #throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    # if len(INIT_V_DATA) > 0:
        # init_data_text.set_text('init point v = %.1f' % INIT_V_DATA[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    # listener()
    cyber.init()
    test_node = cyber.Node("planning_listener")
    test_node.create_reader("/apollo/planning",
                            planning_pb2.ADCTrajectory, callback)

    fig = plt.figure()

    ax_st = plt.subplot2grid((3,3),(0,0),rowspan = 2, colspan = 2)
    ax_sv = plt.subplot2grid((3,3),(0,2),rowspan = 1, colspan = 1)
    ax_sa = plt.subplot2grid((3,3),(1,2),rowspan = 1, colspan = 1)
    ax_sj = plt.subplot2grid((3,3),(2,2),rowspan = 1, colspan = 1)
    ax_speed_limit = plt.subplot2grid((3,3),(2,0),rowspan = 1, colspan = 2)


    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()

    st_curve, = ax_st.plot(
        st_t, st_s, 'g', lw=2, alpha=0.9, label='publish st')

    # obs
    if(plot_st_decider):
        st_decider_obs_curve,= ax_st.plot(st_bounds_decider_obs_t,
            st_bounds_decider_obs_s,
            c='black', alpha=0.8, label='st_decider_obs')

    speed_decider_obs_curve,= ax_st.plot(speed_decider_obs_t,
        speed_decider_obs_s,
        c='red', alpha=0.8, label='speed_bound_prior_decider_obs',linestyle='--')

    speed_decider_s_range_curve,= ax_st.plot(speed_decider_t_range,
        speed_decider_s_range,
        c='black', alpha=0.8, label='path_s_range',linestyle='--')

    speed_decider_final_obs_curve,= ax_st.plot(speed_decider_final_obs_t,
        speed_decider_final_obs_s,
        c='orange', alpha=0.8, label='speed_bound_decider_final_obs',linestyle='-.')

    # drive bound
    if(plot_st_decider):
        st_decider_drive_bound,= ax_st.plot(st_bounds_decider_drive_region_t,
            st_bounds_decider_drive_region_s, c='black', alpha=0.8,
            label='st_decider_drive_region',linestyle='--')
    
    pwj_speed_drive_bound,= ax_st.plot(pwj_speed_drive_region_t,
        pwj_speed_drive_region_s, c='lightgreen', alpha=0.99,
        label='pwj_speed_drive_bound',linestyle='-.')

    # guide_line 
    if(plot_st_decider):
        st_decider_guide_line,= ax_st.plot(st_bounds_decider_guide_line_t,
            st_bounds_decider_guide_line_s, c='black', alpha=0.8,
            label='st_decider_guide_line',linestyle='-.')

    # dp st curve 
    dp_curve,= ax_st.plot(dp_curve_t, dp_curve_s, c='cyan', alpha=0.8,
        label='dp_curve')

    # qp st curve 
    qp_curve,= ax_st.plot(qp_curve_t, qp_curve_s, c='magenta', alpha=0.8,
        label='qp_curve', linestyle='--')

    # nlp st curve 
    nlp_curve,= ax_st.plot(nlp_curve_t, nlp_curve_s, c='blue', alpha=0.8,
        label='nlp_curve', linestyle='-.')

    ax_st.set_xlabel("t")
    ax_st.set_ylabel("s")
    ax_st.set_xlim(-1.0, 10)
    ax_st.set_ylim(-20, 150)
    ax_st.legend(loc="upper right")
    ax_st.grid(linestyle='--')

    # speed plot
    tv_curve, = ax_sv.plot(
        st_t, st_v, 'g', lw=2, alpha=0.9, label='tv')
    dp_tv_curve, = ax_sv.plot(
        dp_curve_t, dp_curve_v, 'r', lw=2, alpha=0.9, label='dp_curve_v')
    qp_tv_curve, = ax_sv.plot(
        qp_curve_t, qp_curve_v, 'b', lw=2, alpha=0.9, label='qp_curve_v')
    nlp_tv_curve, = ax_sv.plot(
        nlp_curve_t, nlp_curve_v, 'purple', lw=2, alpha=0.9, label='nlp_curve_v')


    ax_sv.set_xlabel("t")
    ax_sv.set_ylabel("v")
    ax_sv.set_xlim(-1.0, 10)
    ax_sv.set_ylim(-1, 20)
    ax_sv.grid(linestyle='--')
    ax_sv.legend(loc="upper right")
    

    # acc plot
    sa_curve, = ax_sa.plot(
        st_t, st_acc, 'g', lw=2, alpha=0.99, label='t-acc')

    dp_ta_curve, = ax_sa.plot(
        dp_curve_t, dp_curve_acc, 'r', lw=2, alpha=0.9, label='dp_curve_acc')
    qp_ta_curve, = ax_sa.plot(
        qp_curve_t, qp_curve_acc, 'b', lw=2, alpha=0.9, label='qp_curve_acc')
    nlp_ta_curve, = ax_sa.plot(
        nlp_curve_t, nlp_curve_acc, 'purple', lw=2, alpha=0.9, label='nlp_curve_acc')
    

    ax_sa.set_xlabel("t")
    ax_sa.set_ylabel("acc")
    ax_sa.set_xlim(-1.0, 10)
    ax_sa.set_ylim(-5, 5)
    ax_sa.grid(linestyle='--')
    ax_sa.legend(loc="upper right")
    
    # jerk plot
    sj_curve, = ax_sj.plot(
        st_t, st_jerk, 'g', lw=2, alpha=0.99, label='t-jerk')

    dp_tjerk_curve, = ax_sj.plot(
        dp_curve_t, dp_curve_jerk, 'r', lw=2, alpha=0.99, label='dp jerk')
    qp_tjerk_curve, = ax_sj.plot(
        qp_curve_t, qp_curve_jerk, 'b', lw=2, alpha=0.99, label='qp jerk')
    nlp_tjerk_curve, = ax_sj.plot(
        nlp_curve_t, nlp_curve_jerk, 'purple', lw=2, alpha=0.99, label='nlp jerk')


    ax_sj.set_xlabel("t")
    ax_sj.set_ylabel("jerk")
    ax_sj.set_xlim(-1.0, 10)
    ax_sj.set_ylim(-5, 5)
    ax_sj.grid(linestyle='--')
    ax_sj.legend(loc="upper right")

    # speed limit plot
    speed_limit_curve, = ax_speed_limit.plot(
        speed_limit_s, speed_limit_v, 'r', lw=2, alpha=0.99, label='speed decider prior speed limit')
   
    nlp_speed_limit_curve, = ax_speed_limit.plot(
        nlp_speed_limit_s, nlp_speed_limit_v, 'orange', lw=2, alpha=0.99, label='nlp speed limit')

    sv_curve, = ax_speed_limit.plot(
        st_s, st_v, 'g', lw=2, alpha=0.99, label='s-v')

    ax_speed_limit.set_xlabel("s")
    ax_speed_limit.set_ylabel("v")
    ax_speed_limit.set_xlim(-1, 150)
    ax_speed_limit.set_ylim(-1, 20)
    ax_speed_limit.grid(linestyle='--')
    ax_speed_limit.legend(loc="upper right")



    plt.tight_layout()

    #brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    #throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    # init_data_text = ax_st.set_text(0.75, 0.95, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.legend(loc="upper right")
    plt.show()

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
import math



sk_s = []
sk_wheel = []


lock = threading.Lock()

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Planning plot data length")

plot_st_decider=0

def callback(planning_pb):
    global sk_s
    global sk_wheel



    lock.acquire()


    sk_s = []
    sk_wheel = []

    for traj_point in planning_pb.path_point:

        sk_s.append(traj_point.s)
        kappa = traj_point.kappa

        angle = math.atan(kappa * 2.92) * 180.0 / 3.141592 * 15.7

        sk_wheel.append(angle)


    lock.release()





def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lock.acquire()

    sk_curve.set_xdata(sk_s)
    sk_curve.set_ydata(sk_wheel)


    lock.release()
    #brake_text.set_text('brake = %.1f' % brake_data[-1])
    #throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    # if len(INIT_V_DATA) > 0:
        # init_data_text.set_text('init point v = %.1f' % INIT_V_DATA[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    # listener()
    cyber.init()
    test_node = cyber.Node("path_planning_listener")
    test_node.create_reader("/apollo/planning",
                            planning_pb2.ADCTrajectory, callback)

    fig = plt.figure()

    ax_s_k = plt.subplot2grid((3,3),(0,0),rowspan = 2, colspan = 2)
    ax_sv = plt.subplot2grid((3,3),(0,2),rowspan = 1, colspan = 1)
    ax_sa = plt.subplot2grid((3,3),(1,2),rowspan = 1, colspan = 1)
    ax_sj = plt.subplot2grid((3,3),(2,2),rowspan = 1, colspan = 1)
    ax_speed_limit = plt.subplot2grid((3,3),(2,0),rowspan = 1, colspan = 2)


    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()

    sk_curve, = ax_s_k.plot(
        sk_s, sk_wheel, 'g', lw=2, alpha=0.9, label='s-stering wheel')

    ax_s_k.set_xlabel("s")
    ax_s_k.set_ylabel("path kappa")
    ax_s_k.set_xlim(-20.0, 150)
    ax_s_k.set_ylim(-550, 550)
    ax_s_k.legend(loc="upper right")
    ax_s_k.grid(linestyle='--')


    plt.tight_layout()

    #brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    #throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    # init_data_text = ax_st.set_text(0.75, 0.95, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.legend(loc="upper right")
    plt.show()

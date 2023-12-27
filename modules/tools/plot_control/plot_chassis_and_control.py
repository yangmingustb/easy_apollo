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
import numpy as np

from cyber.python.cyber_py3 import cyber
from modules.canbus.proto import chassis_pb2
from modules.control.proto import control_cmd_pb2

from modules.canbus.proto.chassis_pb2 import Chassis


INIT_ACC_DATA = []
INIT_T_DATA = []
CHASSIS_WHEEL_DATA = []

CONTROL_TIME_DATA = []
CONTROL_ACC_DATA = []
CONTROL_WHEEL_DATA = []

begin_t = None
history_begin_t = None
control_begin_t = None
control_history_begin_t = None
last_t = None
control_last_t = None
last_v = None
control_last_v = None
lock = threading.Lock()

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 6000, "Planning plot data length")


def callback_chassis(chassis_pb):
    global INIT_ACC_DATA, INIT_T_DATA, CHASSIS_WHEEL_DATA
    global begin_t, last_t, last_v, history_begin_t

    if begin_t is None:
        begin_t = chassis_pb.header.timestamp_sec
        last_t = begin_t
    current_t = chassis_pb.header.timestamp_sec
    current_v = chassis_pb.speed_mps
    current_wheel = chassis_pb.steering_percentage / 100.0 * 505
    if((current_t - last_t) <= 0.000001):
        current_acc=(current_v - last_v) / (current_t - last_t)
    else:
        current_acc = 0

    current_acc = min(1, current_acc)
    current_acc = max(-1, current_acc)

    if abs(current_t - last_t) < 0.015:
        return

    lock.acquire()
    if last_t is not None and abs(current_t - last_t) > 1:
        begin_t = chassis_pb.header.timestamp_sec

        INIT_ACC_DATA = []
        INIT_T_DATA = []
        CHASSIS_WHEEL_DATA = []

        last_t = None
        last_v = None

    if len(INIT_T_DATA) > FLAGS.data_length:
        history_begin_t = begin_t
        INIT_T_DATA = INIT_T_DATA[-FLAGS.data_length:]

        begin_t = INIT_T_DATA[0] + history_begin_t
        time_len = len(INIT_T_DATA)

        for i in range(time_len):
            INIT_T_DATA[i] = INIT_T_DATA[i] + history_begin_t - begin_t
    
    if len(INIT_ACC_DATA) > FLAGS.data_length:
        INIT_ACC_DATA = INIT_ACC_DATA[-FLAGS.data_length:]

    if len(CHASSIS_WHEEL_DATA) > FLAGS.data_length:
        CHASSIS_WHEEL_DATA = CHASSIS_WHEEL_DATA[-FLAGS.data_length:]

    if last_t is not None and last_v is not None and current_t > last_t:
        INIT_T_DATA.append(current_t - begin_t)
        INIT_ACC_DATA.append(current_acc)
        CHASSIS_WHEEL_DATA.append(current_wheel)

    lock.release()

    last_t = current_t
    last_v = current_v

def callback_control(control_pb):
    global CONTROL_TIME_DATA, CONTROL_ACC_DATA, CONTROL_WHEEL_DATA
    global control_begin_t, control_last_t, control_last_v, control_history_begin_t

    if control_begin_t is None:
        control_begin_t = control_pb.header.timestamp_sec
        control_last_t = begin_t
    current_t = control_pb.header.timestamp_sec
    current_v = control_pb.speed
    current_wheel = control_pb.steering_target / 100.0 * 505
    current_acc = control_pb.acceleration

    # if abs(current_t - float(control_last_t)) < 0.015:
    #     return

    lock.acquire()
    if control_last_t is not None and abs(current_t - control_last_t) > 1:
        control_begin_t = control_pb.header.timestamp_sec

        CONTROL_TIME_DATA = []
        CONTROL_ACC_DATA = []
        CONTROL_WHEEL_DATA = []

        control_last_t = None
        control_last_v = None


    if len(CONTROL_TIME_DATA) > FLAGS.data_length:
        control_history_begin_t = control_begin_t
        CONTROL_TIME_DATA = CONTROL_TIME_DATA[-FLAGS.data_length:]

        control_begin_t = CONTROL_TIME_DATA[0] + control_history_begin_t
        time_len = len(CONTROL_TIME_DATA)

        for i in range(time_len):
            CONTROL_TIME_DATA[i] = CONTROL_TIME_DATA[i] + control_history_begin_t - control_begin_t
    
    if len(CONTROL_ACC_DATA) > FLAGS.data_length:
        CONTROL_ACC_DATA = CONTROL_ACC_DATA[-FLAGS.data_length:]

    if len(CONTROL_WHEEL_DATA) > FLAGS.data_length:
        CONTROL_WHEEL_DATA = CONTROL_WHEEL_DATA[-FLAGS.data_length:]

    if control_last_t is not None and control_last_v is not None and current_t > control_last_t:
        CONTROL_TIME_DATA.append(current_t - control_begin_t)
        CONTROL_ACC_DATA.append(current_acc)
        CONTROL_WHEEL_DATA.append(current_wheel)

    lock.release()

    control_last_t = current_t
    control_last_v = current_v




def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lock.acquire()

    chassis_acc_curve.set_xdata(INIT_T_DATA)
    chassis_acc_curve.set_ydata(INIT_ACC_DATA)


    control_acc_curve.set_xdata(CONTROL_TIME_DATA)
    control_acc_curve.set_ydata(CONTROL_ACC_DATA)



    chassis_wheel_curve.set_xdata(INIT_T_DATA)
    chassis_wheel_curve.set_ydata(CHASSIS_WHEEL_DATA)

    control_wheel_curve.set_xdata(CONTROL_TIME_DATA)
    control_wheel_curve.set_ydata(CONTROL_WHEEL_DATA)
    lock.release()
    #brake_text.set_text('brake = %.1f' % brake_data[-1])
    #throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    if len(INIT_ACC_DATA) > 0:
        chassis_acc_curve_text.set_text('chassis acc = %.1f' % INIT_ACC_DATA[-1])

    return [chassis_acc_curve, chassis_acc_curve_text]

if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    cyber.init()    
    chassis_sub = cyber.Node("chassis_sub")
    chassis_sub.create_reader('/apollo/canbus/chassis',
                                Chassis, callback_chassis)


    control_sub = cyber.Node("control_sub")
    control_sub.create_reader("/apollo/control",
                            control_cmd_pb2.ControlCommand, callback_control)

    fig = plt.figure()

    # acc
    ax = plt.subplot(1, 2, 1)

    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    chassis_acc_curve, = ax.plot(
        INIT_T_DATA, INIT_ACC_DATA, 'b', lw=2, alpha=0.7, label='chassis acc')
    
    control_acc_curve, = ax.plot(
        CONTROL_TIME_DATA, CONTROL_ACC_DATA, 'g', lw=2, alpha=0.7, label='control acc')

    ax.set_ylim(-8, 8)

    ax.set_xlim(-1, 150)
    ax.legend(loc="upper left")
    ax.grid(linestyle='--')

    # steering angular
    ax2 = plt.subplot(1, 2, 2)

    chassis_wheel_curve, = ax2.plot(
        INIT_T_DATA, CHASSIS_WHEEL_DATA, 'b', lw=2, alpha=0.7, label='chassis wheel')
    
    control_wheel_curve, = ax2.plot(
        CONTROL_TIME_DATA, CONTROL_WHEEL_DATA, 'g', lw=2, alpha=0.7, label='control wheel')

    ax2.set_ylim(-550, 550)


    ax2.set_xlim(-1, 150)
    ax2.legend(loc="upper left")
    ax2.grid(linestyle='--')

    #brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    #throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    chassis_acc_curve_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    # ani = animation.FuncAnimation(fig, update, init_func=init_figure,frames=np.arange(0,100), interval=100)
    ani = animation.FuncAnimation(fig, update, interval=20)

    plt.show()


    # while not cyber.is_shutdown():
    #     print("cyber")

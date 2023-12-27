#!/usr/bin/python3.6
# encoding=utf-8
"""
python tools/plot_log.py -f /apollo/data/log/planning.INFO -t 11:50:34
"""

import argparse
from collections import defaultdict
import os
import re
import shutil
import sys
import time
import math
import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Cursor
from matplotlib.gridspec import GridSpec
from  matplotlib import  ticker

def get_string_between(string, st, ed=''):
    """get string between keywords"""
    if string.find(st) < 0:
        return ''
    sub_string = string[string.find(st) + len(st):]
    if len(ed) == 0 or sub_string.find(ed) < 0:
        return sub_string.strip()
    return sub_string[:sub_string.find(ed)]


def get_planning_seq_num(line):
    """get planning seq num from line"""
    return get_string_between(line, 'start frame sequence id = [', ']')


def get_time(line):
    """get time from line"""
    return get_string_between(line, ' ', ' ')

def get_lines_between(lines, st, ed=''):
    """get valid log with keywords"""
    valid_lines = []
    found_start = False
    for line in lines:
        if st in line:
            found_start = True
        if len(ed) != 0 and ed in line:
            break
        if found_start:
            valid_lines.append(line)
    return valid_lines

def get_single_data_from_line(line, data):
    data.clear()
    x = []

    # .*？ 表示匹配任意个字符到下一个符合条件的字符
    # (.*?) 表示匹配任意个字符到下一个符合条件的字符，且只保留（）中的内容

    # []，匹配集合中的所有字符

    # https://zhuanlan.zhihu.com/p/139596371

    pat = re.compile(r'[(](.*?)[)]', re.S)

    str_list = re.findall(pat, line)
    for string in str_list:
        num = string.split(",")
        x.append(float(num[0]))
    if x:
        data.append(x)
        


def plot_time(lines, ax):


    elem_map = {'prediction_time': [],
                 
                 'planning_time': [],

                 'prepross_time':[],
                 'post_process_time':[],
                 'traj_publish_time':[],
                 'planning_pre_process_time':[],
                 'update_veh_time':[],
                 'traj_stitch_time':[],
                 'init_frame_time':[],
                 'traffic_decider_time':[],

                 'speed_qp_optimization': [],
                 'speed_nlp_optimization': [],
                 'plan_and_prediction_time': [],
                 }

    prediction_time =[]
    planning_time =[]

    prepross_time =[]
    post_process_time =[]
    traj_publish_time =[]


    planning_pre_process_time =[]
    update_veh_time =[]
    traj_stitch_time =[]
    init_frame_time =[]
    traffic_decider_time =[]

    speed_qp_optimization =[]
    speed_nlp_optimization =[]
    plan_and_prediction_time =[]



    for line in lines:
        for key in elem_map.keys():
            name = "print_" + key + ":"
            if name in line:
                get_single_data_from_line(line, elem_map[key])
                # print(elem_map[key][0][0])

                if(key=='prediction_time'):
                    prediction_time.append( elem_map[key][0][0])
                if(key=='planning_time'):
                    planning_time.append( elem_map[key][0][0])

                if(key=='prepross_time'):
                    prepross_time.append(elem_map[key][0][0])
                if(key=='post_process_time'):
                    post_process_time.append(elem_map[key][0][0])

                if(key=='traj_publish_time'):
                    traj_publish_time.append(elem_map[key][0][0])

                if(key=='planning_pre_process_time'):
                    planning_pre_process_time.append(elem_map[key][0][0])

                if(key=='update_veh_time'):
                    update_veh_time.append(elem_map[key][0][0])
                if(key=='traj_stitch_time'):
                    traj_stitch_time.append(elem_map[key][0][0])
                if(key=='init_frame_time'):
                    init_frame_time.append(elem_map[key][0][0])
                if(key=='traffic_decider_time'):
                    traffic_decider_time.append(elem_map[key][0][0])
                if(key=='speed_qp_optimization'):
                    speed_qp_optimization.append(elem_map[key][0][0])
                if(key=='speed_nlp_optimization'):
                    speed_nlp_optimization.append(elem_map[key][0][0])
                if(key=='plan_and_prediction_time'):
                    plan_and_prediction_time.append(elem_map[key][0][0])
    
    # print(control_wheel)

    x = [x+1 for x in range(len(prediction_time))]
    # 纵坐标

    print("prediction size:")
    print(len(x))
    size = len(x)

    for i in range(len(prediction_time)):
        if(len(traj_publish_time) < len(prediction_time)):
            traj_publish_time.append(0)
        else:
            break

    for i in range(len(prediction_time)):
        if(len(planning_pre_process_time) < len(prediction_time)):
            planning_pre_process_time.append(0)
        else:
            break
    for i in range(len(prediction_time)):
        if(len(update_veh_time) < len(prediction_time)):
            update_veh_time.append(0)
        else:
            break
    for i in range(len(prediction_time)):
        if(len(traj_stitch_time) < len(prediction_time)):
            traj_stitch_time.append(0)
        else:
            break
    for i in range(len(prediction_time)):
        if(len(init_frame_time) < len(prediction_time)):
            init_frame_time.append(0)
        else:
            break
    for i in range(len(prediction_time)):
        if(len(traffic_decider_time) < len(prediction_time)):
            traffic_decider_time.append(0)
        else:
            break

    for i in range(len(prediction_time)):
        if(len(speed_qp_optimization) < len(prediction_time)):
            speed_qp_optimization.append(0)

        if(len(speed_nlp_optimization) < len(prediction_time)):
            speed_nlp_optimization.append(0)

        if(len(plan_and_prediction_time) < len(prediction_time)):
            plan_and_prediction_time.append(0)


    planning_pre_process_time = planning_pre_process_time[0:  size]
    print("planning_pre_process_time size:")
    print(len(planning_pre_process_time))

    print("traffic_decider_time size:")
    print(len(traffic_decider_time))
    
    print("speed_qp_optimization size:")
    print(len(speed_qp_optimization))

    while(len(plan_and_prediction_time) > len(prediction_time)):
        plan_and_prediction_time.pop()

    # 生成折线图：函数polt
    # plt.plot(x, y)    




    for key in elem_map.keys():
        if elem_map[key]:
            if(key =='prediction_time'):
                y = prediction_time
                ax["time"].plot(x,y, label= key, c='limegreen',linestyle='-')
            if(key =='planning_time'):
                y = planning_time
                ax["time"].plot(x,y, label= key,c='magenta',linestyle='-',linewidth=1)
            
            if(key =='prepross_time'):
                y = prepross_time
                ax["time"].plot(x,y, label= key,c='orange',linestyle='--')
            
            if(key =='post_process_time'):
                y = post_process_time
                ax["time"].plot(x,y,  label= key,c='blue',linestyle='--')

            if(key =='traj_publish_time'):
                y = traj_publish_time
                ax["time"].plot(x,y,  label= key,c='red',linestyle='-')

            if(key =='planning_pre_process_time'):
                y = planning_pre_process_time
                ax["time"].plot(x,y,  label= key,c='black')
            
            if(key =='traffic_decider_time'):
                y = traffic_decider_time
                ax["time"].plot(x,y,  label= key)

            if(key =='speed_qp_optimization'):
                y = speed_qp_optimization
                ax["time"].plot(x,y,  label= key)

            if(key =='speed_nlp_optimization'):
                y = speed_nlp_optimization
                ax["time"].plot(x,y,  label= key)

            if(key =='plan_and_prediction_time'):
                y = plan_and_prediction_time
                ax["time"].plot(x,y,  label= key)

            if(key =='init_frame_time'):
                y = init_frame_time
                ax["time"].plot(x,y,  label= key)
            
            if(key =='traj_stitch_time'):
                y = traj_stitch_time
                ax["time"].plot(x,y,  label= key)

            if(key =='update_veh_time'):
                y = update_veh_time
                ax["time"].plot(x,y,  label= key)

    
    ax["time"].set_ylim(0, 1000)
    ax["time"].set_xlabel("id")
    ax["time"].set_ylabel("time:ms")



def plot_frame(fig, ax, lines, line_st_num, line_ed_num):
    """plot ref frame"""
    print( 'plot line start num: ' + str(line_st_num + 1))
    print( 'plot line end num: ' + str(line_ed_num + 1))
    frame_seq = get_planning_seq_num(lines[line_st_num])
    print( 'frame seq num: ' + frame_seq)


    valid_lines = []
    for i in range(line_st_num, line_ed_num):
        valid_lines.append(lines[i])

    # plot curve from point vectors
    ax_title = 'seq: ' + frame_seq
    fig.suptitle(ax_title)
    for value in ax.values():
        value.lines = []
        value.texts = []
    plot_acc(valid_lines, ax)

    for value in ax.values():
        value.legend()
        value.grid(True)

    #ax.set_aspect(1)
    
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    plt.draw()

    return




def search_next(lines, line_search_num):
    """search forward, return frame start and end line number"""
    start_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, len(lines)):
        if 'Planning start frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            start_line_num = i
            break
    if start_line_num < 0:
        return -1, -1, seq_id

    for i in range(start_line_num, len(lines)):
        if 'Planning end frame sequence id = [' + seq_id in lines[i]:
            return start_line_num, i, seq_id
    return start_line_num, -1, seq_id


def search_last(lines, line_search_num):
    end_line_num = -1
    seq_id = '-1'
    for i in range(line_search_num, 0, -1):
        if 'Planning end frame sequence id' in lines[i]:
            seq_id = get_string_between(lines[i], 'sequence id = [', ']')
            end_line_num = i
            break
    if end_line_num < 0:
        return -1, -1, seq_id

    for i in range(end_line_num, 0, -1):
        if 'Planning start frame sequence id = [' + seq_id in lines[i]:
            return i, end_line_num, seq_id
    return -1, end_line_num, seq_id

def search_time_line(lines, search_time):
    """search line with time"""
    for i in range(len(lines)):
        if search_time in lines[i]:
            return i + 1
    return 0


def search_seq_line(lines, search_seq):
    """search line with time"""
    for i in range(len(lines)):
        if 'start frame sequence id' in lines[i]:
            if 'start frame sequence id = [' + search_seq in lines[i]:
                return i + 1
    return 0


class MouseEventManager(object):
    x, y = 0.0, 0.0
    xoffset, yoffset = -20, 20
    text_template = 'x: %0.2f\ny: %0.2f'
    annotation = False

    def on_click(self, event):
        # if mouse button is not right, return
        # 1: left, 2: middle, 3: right
        if event.button is not 3:
            return
        self.x, self.y = event.xdata, event.ydata
        if self.x is not None:
            print( 'mouse click x: %.2f, y: %.2f' % (event.xdata, event.ydata))
            if self.annotation:
                self.annotation.set_visible(False)
            label_text = self.text_template % (self.x, self.y)
            self.annotation = event.inaxes.annotate(label_text,
                xy=(self.x, self.y), xytext=(self.xoffset, self.yoffset),
                textcoords='offset points', ha='right', va='bottom',
                bbox=dict(boxstyle='round,pad=0.5', fc='lightcyan', alpha=0.5),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0')
                )
            self.annotation.set_visible(True)
            self.annotation.figure.canvas.draw()


class Index(object):
    """button callback function"""
    def __init__(self,fig, ax, line_st_num, line_ed_num, lines):
        self.ax = ax
        self.fig = fig
        self.line_st_num = line_st_num
        self.line_ed_num = line_ed_num
        self.lines = lines
        self.reset_mouse_event()

    def reset_mouse_event(self):
        self.mouse_manager = MouseEventManager()
        fig.canvas.mpl_connect('button_release_event', self.mouse_manager.on_click)

    def next(self, step):
        """next button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id =\
                    search_next(self.lines, self.line_ed_num + 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print( '[ERROR] search reach last line, may reach last frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines, self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def next1(self, event):
        self.next(1)

    def next10(self, event):
        self.next(10)

    def prev(self, step):
        """prev button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id =\
                    search_last(self.lines, self.line_st_num - 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach first line, may reach first frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines, self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def prev1(self, event):
        self.prev(1)

    def prev10(self, event):
        self.prev(10)

    def exit(self, event):
        sys.exit(0)



if __name__ == '__main__':
    global g_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', action='store', dest='log_file_path', required=True, help='log file path')
    parser.add_argument('-t', action='store', dest='time', required=False, help='time begin to search')
    parser.add_argument('-ut', action='store', dest='unix_time', required=False, help='unix time begin to search')
    parser.add_argument('-s', action='store', dest='seq', required=False, help='sequence number to search')
    g_argv = parser.parse_args()
    print('Please wait for loading data...')

    # load log file
    print (g_argv)
    file_path = g_argv.log_file_path
    #file_path = parse_pb_log(file_path)
    search_time = g_argv.time
    search_seq = g_argv.seq
    unix_time = g_argv.unix_time

    input = open(file_path, 'r')

    lines = input.readlines()

    line_search_num =0
    if search_time:
        line_search_num = search_time_line(lines, search_time)
    elif search_seq:
        line_search_num = search_seq_line(lines, search_seq)
    elif  unix_time:
        search_time = datetime.utcfromtimestamp(float(unix_time)).strftime('%H:%M:%S')
        print("from unixtime %s to data time %s"%(unix_time, search_time))
    else:
        print( 'search time or sequence number or unix time is required, quit!')

    print( "line number: ")
    print( line_search_num)
    # # check whether time is exist
    # if line_search_num == 0:
    #     print( 'no such time, quit!')
    #     sys.exit(0)

    # line_st_num, line_ed_num, seq_id = search_next(lines, line_search_num)
    # # check whether found frame log is complete
    # if line_st_num < 0 or line_ed_num < 0:
    #     print( '[ERROR] search reach last line, may reach last frame, quit!')
    #     sys.exit(0)

    fig = plt.figure(figsize = [9, 15])
    gs = GridSpec(1, 1, figure=fig)
    ax = {}
    ax['time'] = fig.add_subplot(gs[0,0])

    # fig2 = plt.figure(figsize = [9, 15])
    # gs2 = GridSpec(2, 1, figure=fig2)
    # ax2 = {}
    # ax2['speed'] = fig2.add_subplot(gs2[0,0])
    # ax2['lat_offset'] = fig2.add_subplot(gs2[1,0])

    # plot_frame(fig, ax, lines, line_st_num, line_ed_num)

    # plot_acc( lines, ax)
    plot_time( lines, ax)
    for value in ax.values():
        value.legend()
        value.grid(True)


    #ax.set_aspect(1)
    
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)


    # 显示网格
    plt.grid(True)
    # 显示图表
    plt.show()
  


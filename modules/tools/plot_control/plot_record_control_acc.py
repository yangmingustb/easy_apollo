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

import math
from modules.tools.plot_planning.record_reader import RecordItemReader


class control_record:

    def __init__(self):
        self.timestamp_list = []
        self.speed_list = []
        self.acc_list = []
        self.steering_list = []

    def add(self, control):
        timestamp_sec = control.header.timestamp_sec
        speed_mps = control.speed
        acc = control.acceleration
        steering_target = control.steering_target

        self.timestamp_list.append(timestamp_sec)
        self.speed_list.append(speed_mps)
        self.acc_list.append(acc)
        self.steering_list.append(steering_target)

    def get_speed_list(self):
        return self.speed_list

    def get_acc_list(self):
        return self.acc_list
    
    def get_steering_list(self):
        return self.steering_list

    def get_timestamp_list(self):
        return self.timestamp_list

    def get_lastest_speed(self):
        if len(self.speed_list) > 0:
            return self.speed_list[-1]
        else:
            return None

    def get_lastest_timestamp(self):
        if len(self.timestamp_list) > 0:
            return self.timestamp_list[-1]
        else:
            return None


if __name__ == "__main__":
    import sys
    import matplotlib.pyplot as plt
    from os import listdir
    from os.path import isfile, join

    # 参数列表
    # sys.argv[0]表示文件名
    folders = sys.argv[1:]

    fig, ax = plt.subplots()
    colors = ["g", "b", "r", "m", "y"]
    markers = ["o", "o", "o", "o"]
    for i in range(len(folders)):
        folder = folders[i]
        print(folder)

        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        fns = [f for f in listdir(folder) if isfile(join(folder, f))]

        print(fns)

        for fn in fns:
            reader = RecordItemReader(folder+"/"+fn)

            processor = control_record()
            
            last_control_data = None
            
            topics = ["/apollo/control"]
            
            for data in reader.read(topics):
                if "control" in data:
                    last_control_data = data["control"]
                    processor.add(last_control_data)
                    last_pose_data = None
                    last_chassis_data = None

            # data_x = processor.get_timestamp_list()
            data_x = processor.get_timestamp_list()
            print(len(data_x))
            if(len(data_x) <=0):
                continue

            x_list = [x for x in range(len(data_x))]
            data_y = processor.get_acc_list()

            # ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)
            # ax.plot(x_list, data_y, alpha=1, label='control acc: '+fn)
            ax.plot(data_x, data_y, alpha=1, label='control acc: '+fn)

    ax.set_xlabel("id")
    ax.set_ylabel("control acc")

    # 显示网格
    plt.legend(loc='best')
    plt.grid(True)
    plt.show()

#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${TOP_DIR}/set_python_env.bash"

# https://blog.csdn.net/weixin_64338372/article/details/128073840?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-128073840-blog-113502992.235^v38^pc_relevant_anti_vip_base&spm=1001.2101.3001.4242.1&utm_relevant_index=3

# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/test_data/control.log_mpc.INFO

# simulate pp
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_180157/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231011_001150/control.INFO

# add pp limit
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231011_094441/control.INFO

# add centric acc limit
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231011_105346/control.INFO

# pp, no limit
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231011_095512/control.INFO

# simulate mpc
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_180645/control.INFO

# real car mpc
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_171119/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_171506/control.INFO


# real car pp
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_170834/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_170630/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231010_170310/control.INFO

# real car pp, 直线行驶时，方向盘振荡
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231019_114217/control.INFO



# simulate, pp, 调试锯齿

# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231016_185754/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231020_140545/control.INFO

# real car pp， 调试锯齿
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231019_114217/control.INFO


# simulator pp, add chassis delay
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231020_163914/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231023_18/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231024_10/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231024_11/control.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231026_15/control.INFO


# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231208_11/control.log.INFO.20231208-111001.8690
# python3 ./modules/tools/plot_controlplot_control/plot_info_control.py -f ./data/log/20231220_15/control.log.INFO.20231220-153737.51043
python3 ./modules/tools/plot_control/plot_info_control.py -f ./data/log/20231225_09/control.log.INFO.20231225-095737.19692
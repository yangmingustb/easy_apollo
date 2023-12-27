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
# python3 ./modules/tools/plot_controlplot_planning/plot_log_planning_time.py -f ./data/log/20231117_10/planning.log.INFO.20231117-102452.12647
python3 ./modules/tools/plot_planning/plot_log_planning_time.py -f ./data/log/20231214_10/planning.log.INFO.20231214-104557.86067


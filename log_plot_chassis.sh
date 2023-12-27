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

# python3 ./modules/tools/plot_controlplot_control/plot_info_can.py -f ./data/log/20231016_114441/chassis.INFO
# python3 ./modules/tools/plot_controlplot_control/plot_info_can.py -f ./data/log/20231218_16/chassis.log.INFO.20231218-165331.222488
# python3 ./modules/tools/plot_controlplot_control/plot_info_can.py -f ./data/log/20231010_170310/chassis.INFO
python3 ./modules/tools/plot_control/plot_info_can.py -f ./data/log/20231220_15/chassis.log.INFO.20231220-153737.51044

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

# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/apollo_map/apollo/town01/base_map.bin -signal -junction
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/apollo_map/apollo/town07/base_map.bin -sl -signal -junction
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/yuanqu/base_map.bin -sl -signal -stopsign 
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/shuiku_double_lane/base_map.bin -sl -signal -stopsign 
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/apollo_virutal_map/base_map.bin -sl -signal -stopsign 
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/apollo_virutal_map/base_map.bin -sl -signal -stopsign -yiledsign -junction -crosswalk --position
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/demo/base_map.txt -sl
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/borregas_ave/base_map.bin -sl -junction -crosswalk -signal
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/g2_g3/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/jiapulin_9_20/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/sanhuan/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/mymap/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/shuiku_double_lane_dead_end/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/shuiku_double_direction/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/shuiku_backward/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/shuiku_forward/base_map.bin -sl -junction -crosswalk
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/sunnyvale/base_map.bin -sl
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/apollo_virutal_map/base_map.bin -sl
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/jiapuling_double_lane_6_27/base_map.bin -sl
# python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/jiapuling_intersection/base_map.bin -sl
python3  ./src/modules/tools/mapshow/mapshow.py -m ./data/sanhuan_back/base_map.bin

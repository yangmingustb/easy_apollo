#!/bin/bash
#

source ./set_env.bash
source ./setup.bash




# run replay file
# ./cyber_recorder play -f ./../data/log/20231031_16/20231031165028.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231026_17/20231026173052.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231107_15/20231107154131.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231107_15/20231107153701.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231107_15/20231107152643.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231107_15/20231107151847.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231122_15/20231122151401.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231128_10/20231128105513.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231128_19/20231128191929.record.00000 \

# ./cyber_recorder play -f ./../data/log/20231212_11/20231212112325.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231205_14/20231205145940.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231217_18/20231217181810.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231218_15/20231218154001.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231220_14/20231220142736.record.00000 \
# ./cyber_recorder play -f ./../data/log/20231220_14/20231220145008.record.00000 \
./cyber_recorder play -f ./../data/log/20231220_15/20231220153952.record.00000 \
  -k /apollo/planning \
  -k /apollo/prediction \
  -k /apollo/routing_response \
  -k /apollo/control \
  -s 61
  # -b 2023-12-05-15:01:50  


# Hit Ctrl+C to stop, Space to pause, or 's' to step.
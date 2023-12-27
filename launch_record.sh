#!/bin/bash
#

source ./set_env.bash
source ./setup.bash



./cyber_recorder record -a \
                        -k /apollo/planning \
                        -k /apollo/prediction \
                        -k /apollo/routing_response \
                        -k /apollo/control \
                        -i 600 \
                        -m 100

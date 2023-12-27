set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[-1]}" )" && pwd )"

INSTALL_SPACE="build"

cd ${SCRIPT_DIR}  
cd ${INSTALL_SPACE}
source ./../install/setup.bash 
# source ${INSTALL_SPACE}/set_env.bash
# source ${INSTALL_SPACE}/setup.bash

cmake ..
# make -j6 -DARM=0 -DCMAKE_INSTALL_PREFIX=${INSTALL_SPACE}
make -j6
make install
# cd ${SCRIPT_DIR}/${INSTALL_SPACE} && ./encrypt_param.sh
# cd ${SCRIPT_DIR}/${INSTALL_SPACE}/etc && rm *.json




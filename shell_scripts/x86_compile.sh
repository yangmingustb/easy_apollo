set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[-1]}" )" && pwd )"

INSTALL_SPACE="install"

cd ${SCRIPT_DIR}  && source ${INSTALL_SPACE}/setup.bash
catkin_make install -DARM=0 -DUVIZ=0 -DCMAKE_INSTALL_PREFIX=${INSTALL_SPACE}
cd ${SCRIPT_DIR}/${INSTALL_SPACE} && ./encrypt_param.sh
cd ${SCRIPT_DIR}/${INSTALL_SPACE}/etc && rm *.json


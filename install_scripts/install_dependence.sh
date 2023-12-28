


set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[-1]}" )" && pwd )"

bash ${SCRIPT_DIR}/install_cmake.sh
bash ${SCRIPT_DIR}/install_g++.sh
bash ${SCRIPT_DIR}/install_git.sh
bash ${SCRIPT_DIR}/install_poco.sh

bash ${SCRIPT_DIR}/install_gtest.sh
bash ${SCRIPT_DIR}/install_glog.sh
bash ${SCRIPT_DIR}/install_gflags.sh

bash ${SCRIPT_DIR}/install_protobuf.sh

bash ${SCRIPT_DIR}/install_fastrtps_fastcdr.sh

bash ${SCRIPT_DIR}/install_pcl.sh

bash ${SCRIPT_DIR}/install_civetweb.sh

bash ${SCRIPT_DIR}/install_yaml_cpp.sh

bash ${SCRIPT_DIR}/install_tinyxml2.sh


bash ${SCRIPT_DIR}/install_ncurses.sh

bash ${SCRIPT_DIR}/install_can.sh

bash ${SCRIPT_DIR}/install_adolc.sh

bash ${SCRIPT_DIR}/install_ipopt.sh

bash ${SCRIPT_DIR}/install_opengl.sh

bash ${SCRIPT_DIR}/install_opencv.sh

bash ${SCRIPT_DIR}/install_abseil.sh

bash ${SCRIPT_DIR}/install_ad_rss_lib.sh

bash ${SCRIPT_DIR}/install_eigen.sh

bash ${SCRIPT_DIR}/install_libtorch.sh

#!/usr/bin/env bash

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

set -e


CURRENT_PATH=$(cd $(dirname $0) && pwd)

INSTALL_PATH="$CURRENT_PATH/../third_party/"

cd $CURRENT_PATH
cd ..
cd third_party


echo "############### Build Fast-DDS. ################"

PKG_NAME="fast-rtps-1.5.0-1.prebuilt.x86_64.tar.gz"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
wget -t 10 $DOWNLOAD_LINK -P $INSTALL_PATH

tar -zxf ${PKG_NAME}

cd install
mkdir fastrtps

cd ..
cp -r fast-rtps-1.5.0-1/* ./install/fastrtps




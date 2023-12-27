#!/usr/bin/env bash
set -e

declare -a DIRS=("apollo_interface" "can" "common" "can_test" "hdmap_tools"
      "virtual_chassis" "replay" "test" "tools" "display" "auto_components"
      "modules" "cyber" "control" "chassis")

for val in ${DIRS[@]}; do 
  echo ${val}
  echo "Formatting codes in ${val} directories ....."
  find ./src/${val} -iname '*.h' ! -iname "*.pb.h" | xargs clang-format -i
  find ./src/${val} -iname '*.hpp' | xargs clang-format -i
  find ./src/${val} -iname '*.cc' ! -iname "*.pb.cc" | xargs clang-format -i
  find ./src/${val} -iname '*.cpp' | xargs clang-format -i
  echo "Done!"
done


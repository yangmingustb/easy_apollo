#!/usr/bin/env bash
set -e

declare -a DIRS=(
      "test" "tools"
      "modules" "cyber")

for val in ${DIRS[@]}; do 
  echo ${val}
  echo "Formatting codes in ${val} directories ....."
  find ./${val} -iname '*.h' ! -iname "*.pb.h" | xargs clang-format -i
  find ./${val} -iname '*.hpp' | xargs clang-format -i
  find ./${val} -iname '*.cc' ! -iname "*.pb.cc" | xargs clang-format -i
  find ./${val} -iname '*.cpp' | xargs clang-format -i
  echo "Done!"
done


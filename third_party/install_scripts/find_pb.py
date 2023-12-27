
from glob import glob
import os
import argparse
import json
import xml.etree.ElementTree as ET
from typing import Dict, List
import re
from shutil import copyfile


def main():

    base_dir = "/apollo/share/modules"
    pb_h_dir = "/apollo/.cache/bazel/540135163923dd7d5820f3ee4b306b32/execroot/apollo/bazel-out/k8-dbg/bin/modules"


    #所有的pb.h文件
    pb_h_paths = glob(pb_h_dir + "/**/*.pb.h", recursive=True)

    #将文件按照目录结构写到指定位置
    for pb_h_path in pb_h_paths:
        print(pb_h_path)
        rel_path = pb_h_path[len(pb_h_dir):]
        rel_dir, pb_h_name = os.path.split(rel_path)
        abs_dir = base_dir + rel_dir

        #如果目录不存在，则创建
        if not os.path.exists(abs_dir):
            os.makedirs(abs_dir)

        copyfile(pb_h_path, abs_dir + "/" + pb_h_name)

        pb_cc_path = pb_h_path.replace(".pb.h", ".pb.cc")
        pb_cc_name = pb_h_name.replace(".pb.h", ".pb.cc")
        copyfile(pb_h_path, abs_dir + "/" + pb_h_name)
        copyfile(pb_cc_path, abs_dir + "/" + pb_cc_name)

    
    print("ok")
    return


if __name__ == '__main__':
    main()



    
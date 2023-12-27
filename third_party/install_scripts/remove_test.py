
from glob import glob
import os
import argparse
import json
import xml.etree.ElementTree as ET
from typing import Dict, List
import re, shutil



def main():

    base_dir = "/home/qkj/project/auto/noa/nop_baidu/src/modules/prediction"
    

    #所有的pb.h文件
    test_cc_paths = glob(base_dir + "/**/*_test.cc", recursive=True)

    #将文件按照目录结构写到指定位置
    for test_cc_path in test_cc_paths:
        print(test_cc_path)

        os.remove(test_cc_path)

    
    print("ok")
    return


if __name__ == '__main__':
    main()



    
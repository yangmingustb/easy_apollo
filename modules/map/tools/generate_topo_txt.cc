/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "gflags/gflags.h"

#include "modules/map/proto/map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/proto/topo_graph.pb.h"

/**
 * A map tool to transform .txt map to .bin map
 */


int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    google::ParseCommandLineFlags(&argc, &argv, true);

    // FLAGS_map_dir = "./../data/sunnyvale_big_loop";
    // FLAGS_map_dir = "./../data/sunnyvale";
    FLAGS_map_dir = "./../data/borregas_ave";
    // FLAGS_map_dir = "./../data/apollo_virutal_map";

    const auto map_filename = FLAGS_map_dir + "/routing_map.bin";

    apollo::routing::Graph graph;
    if (!apollo::cyber::common::GetProtoFromBinaryFile(map_filename, &graph))
    {
        AERROR << "Failed to load bin map from " << map_filename;
        return -1;
    }
    else
    {
        AINFO << "Loaded bin map from " << map_filename;
    }

    const std::string output_txt_file = FLAGS_map_dir + "/routing_map.txt";
    if (!apollo::cyber::common::SetProtoToASCIIFile(graph, output_txt_file))
    {
        AERROR << "Failed to generate binary base map";
        return -1;
    }

    graph.Clear();
    ACHECK(apollo::cyber::common::GetProtoFromASCIIFile(output_txt_file, &graph))
            << "Failed to load generated binary base map";

    AINFO << "Successfully converted .bin map to .txt map: " << output_txt_file;

    return 0;
}

/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::PointCloud;
using pcl::PointXYZ;

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

void MessageCallback(
        const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg)
{
    AINFO << "Received message seq-> " << msg->seq();
    AINFO << "msgcontent->" << msg->content();
}

int main(int argc, char* argv[])
{
    apollo::cyber::binary::SetName("test talker");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    // create talker node
    auto node1 = apollo::cyber::CreateNode("talker");

    // create talker
    auto talker = node1->CreateWriter<Chatter>("channel/chatter");

    // read
    auto reader = node1->CreateReader<apollo::cyber::examples::proto::Chatter>(
            "channel/chatter2", MessageCallback);

    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(20, 32, 45);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud->points.resize(4);

    (*cloud)[0].x = 10;
    (*cloud)[0].y = -100;
    (*cloud)[0].z = 0.0f;

    (*cloud)[1].x = 10;
    (*cloud)[1].y = 140;
    (*cloud)[1].z = 0.0f;

    (*cloud)[2].x = -10;
    (*cloud)[2].y = 140;
    (*cloud)[2].z = 0.0f;

    (*cloud)[3].x = -10;
    (*cloud)[3].y = -100;
    (*cloud)[3].z = 0.0f;

    viz.addPolygon<PointXYZ>(cloud, 1.0, 0.0, 0.0, "polygon");

    viz.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "polygon");

    double camera_pos[3] = {0, -100, 30};
    double view_pos[3] = {0, -100, 30};

    view_pos[1] += 50;
    view_pos[2] -= 30;

    viz.setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],
                          view_pos[0], view_pos[1], view_pos[2], 0, 0, 0);

    printf("debug1\n");

    // 秒
    Rate rate(10.0);
    for (int i = 0; i < 10000; i++)
    {
    
        static uint64_t seq = 0;
        auto msg = std::make_shared<Chatter>();
        msg->set_timestamp(Time::Now().ToNanosecond());
        msg->set_lidar_timestamp(Time::Now().ToNanosecond());
        msg->set_seq(seq++);
        msg->set_content("Hello, apollo! I am node_1");

        talker->Write(msg);

        AINFO << "node1 sent a message!";
        AINFO << msg->DebugString();

        printf("cycle %d\n", i);

        camera_pos[1] +=1;

        view_pos[1] += 1;

        viz.setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],
                              view_pos[0], view_pos[1], view_pos[2], 0, 0, 0);

        double a;

        rate.Sleep();

        viz.setBackgroundColor(20, 32, 45);
        viz.spinOnce(10);
    }

    viz.spin();
    viz.close();

    apollo::cyber::WaitForShutdown();
    return 0;
}

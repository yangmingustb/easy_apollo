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

void MessageCallback(
        const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg)
{
    AINFO << "Received message seq-> " << msg->seq();
    AINFO << "msgcontent->" << msg->content();
}

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

int main(int argc, char* argv[])
{
    apollo::cyber::binary::SetName("test listener");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());
    // create listener node
    auto node2 = apollo::cyber::CreateNode("listener");

    auto writter = node2->CreateWriter<Chatter>("channel/chatter2");

    // create listener
    auto listener =
            node2->CreateReader<apollo::cyber::examples::proto::Chatter>(
                    "channel/chatter", MessageCallback);

    // writter
    Rate rate(1.0);
    uint64_t seq = 0;
    while (apollo::cyber::OK())
    {
        auto msg = std::make_shared<Chatter>();
        msg->set_timestamp(Time::Now().ToNanosecond());
        msg->set_lidar_timestamp(Time::Now().ToNanosecond());
        msg->set_seq(seq);
        msg->set_content("Hello, apollo, I am node_2!");
        writter->Write(msg);

        AINFO << "node2 sent a message! No. " << seq;
        AINFO << msg->DebugString();

        seq++;
        rate.Sleep();
    }

    apollo::cyber::WaitForShutdown();
    return 0;
}

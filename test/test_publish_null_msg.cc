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

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

void MessageCallback(
        const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg)
{
    AINFO << "Received message seq-> " << msg->seq();
    AINFO << "msgcontent->" << msg->content();
}

// 测试发布空消息，会发生什么。
// 可以正常发送空消息
int main(int argc, char* argv[])
{
    apollo::cyber::binary::SetName("test talker");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    // create talker node
    auto node1 = apollo::cyber::CreateNode("talker");

    // create talker
    auto talker = node1->CreateWriter<Chatter>("channel/chatter");

    // 秒
    Rate rate(10.0);
    while (apollo::cyber::OK())
    {
        static uint64_t seq = 0;
        auto msg = std::make_shared<Chatter>();
        // msg->set_timestamp(Time::Now().ToNanosecond());
        // msg->set_lidar_timestamp(Time::Now().ToNanosecond());
        // msg->set_seq(seq++);
        // msg->set_content("Hello, apollo! I am node_1");
        msg->Clear();

        talker->Write(msg);

        AINFO << "node1 sent a message!";
        AINFO << msg->DebugString();

        rate.Sleep();
    }

    apollo::cyber::WaitForShutdown();
    return 0;
}

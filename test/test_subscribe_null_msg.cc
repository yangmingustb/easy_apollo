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

apollo::cyber::examples::proto::Chatter msg_;

void MessageCallback(
        const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg)
{
    msg_.CopyFrom(*msg);
    AINFO << msg_.DebugString();
    AINFO << "Received message seq-> " << msg_.seq();
    AINFO << "msgcontent->" << msg_.content();

    std::string content = msg_.content();

    std::cout << content;
}

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

// 测试接收到空消息，发生什么.
// 可以正常接收消息
// 即使是空的，使用没有报错
int main(int argc, char* argv[])
{
    apollo::cyber::binary::SetName("test listener");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());
    // create listener node
    auto node2 = apollo::cyber::CreateNode("listener");

    msg_.Clear();

    // create listener
    auto listener =
            node2->CreateReader<apollo::cyber::examples::proto::Chatter>(
                    "channel/chatter", MessageCallback);

    // writter
    Rate rate(1.0);
    uint64_t seq = 0;
    while (apollo::cyber::OK())
    {
        seq++;
        rate.Sleep();
    }

    apollo::cyber::WaitForShutdown();
    return 0;
}

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

#include "cyber/examples/proto/examples.pb.h"

#include "cyber/cyber.h"

using apollo::cyber::examples::proto::Driver;

int main(int argc, char* argv[])
{
    apollo::cyber::Init(argv[0]);
    std::string server_name = "example_server";
    std::shared_ptr<apollo::cyber::Node> node(
            apollo::cyber::CreateNode("server_node"));

    auto server = node->CreateService<Driver, Driver>(
            server_name, [](const std::shared_ptr<Driver>& request,
                            std::shared_ptr<Driver>& response) {
                AINFO << "server: i am driver server";
                static uint64_t id = 0;
                response->set_msg_id(id);
                id++;
            });

    apollo::cyber::WaitForShutdown();
    return 0;
}

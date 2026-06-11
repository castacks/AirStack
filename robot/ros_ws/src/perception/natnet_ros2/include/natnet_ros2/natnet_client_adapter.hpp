// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// natnet_client_adapter.hpp — declaration of NatNetClientAdapter.
//
// NatNetClientAdapter wraps the NatNet SDK's NatNetClient and implements
// INatNetClient.  It is the only place in the codebase that includes NatNet
// SDK headers; unit tests use FakeNatNetClient instead.
//
// Implementation: src/natnet_client_adapter.cpp

#pragma once

#include "natnet_ros2/natnet_logic.hpp"

#include <functional>
#include <memory>
#include <vector>

// Forward-declare the SDK type so this header stays SDK-header-free.
class NatNetClient;

namespace natnet_ros2
{

class NatNetClientAdapter : public INatNetClient
{
public:
    NatNetClientAdapter();
    ~NatNetClientAdapter() override;

    NatNetResult connect(const ConnectConfig & cfg) override;
    bool get_server_info(ServerInfo & out) override;
    std::vector<BodyDescriptor> get_body_descriptors() override;
    void set_frame_callback(std::function<void(const FrameSample &)> cb) override;
    void disconnect() override;

private:
    std::unique_ptr<NatNetClient> client_;
    std::function<void(const FrameSample &)> user_cb_;
};

}  // namespace natnet_ros2

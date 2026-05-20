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
// fake_natnet_client.hpp — in-process test double for INatNetClient.
//
// Used exclusively in unit tests (test_natnet_logic.cpp).
// Never included in production binaries.
//
// Usage:
//   FakeNatNetClient fake;
//   fake.connect_result    = NatNetResult::OK;
//   fake.server_info       = { .host_present = true, .host_app_name = "Motive" };
//   fake.body_descriptors  = {{ .id=1, .name="Drone", .parent_id=-1 }};
//
//   auto result = natnet_ros2::negotiate(fake, cfg);
//   EXPECT_TRUE(result.ok);

#pragma once

#include "natnet_ros2/natnet_logic.hpp"

#include <functional>
#include <string>
#include <vector>

namespace natnet_ros2
{

class FakeNatNetClient : public INatNetClient
{
public:
    // -----------------------------------------------------------------------
    // Configurable behaviour — set before calling negotiate() / testing
    // -----------------------------------------------------------------------

    /// What connect() should return.
    NatNetResult connect_result = NatNetResult::OK;

    /// What get_server_info() should populate and return.
    /// Set host_present = true to simulate a fully-identified server.
    ServerInfo server_info;

    /// get_server_info() return value (independent of server_info.host_present,
    /// so tests can simulate "SDK call failed" vs. "host not present").
    bool server_info_call_succeeds = true;

    /// What get_body_descriptors() should return.
    std::vector<BodyDescriptor> body_descriptors;

    // -----------------------------------------------------------------------
    // Call-record state — inspect after exercising the fake
    // -----------------------------------------------------------------------

    bool        connect_was_called        = false;
    ConnectConfig last_connect_config;

    bool        server_info_was_called    = false;
    bool        descriptors_was_called    = false;
    bool        set_callback_was_called   = false;
    bool        disconnect_was_called     = false;

    /// Frames that were injected via inject_frame().
    int         frames_injected           = 0;

    // -----------------------------------------------------------------------
    // INatNetClient overrides
    // -----------------------------------------------------------------------

    NatNetResult connect(const ConnectConfig & cfg) override
    {
        connect_was_called  = true;
        last_connect_config = cfg;
        return connect_result;
    }

    bool get_server_info(ServerInfo & out) override
    {
        server_info_was_called = true;
        out = server_info;
        return server_info_call_succeeds;
    }

    std::vector<BodyDescriptor> get_body_descriptors() override
    {
        descriptors_was_called = true;
        return body_descriptors;
    }

    void set_frame_callback(std::function<void(const FrameSample &)> cb) override
    {
        set_callback_was_called = true;
        frame_cb_ = cb;
    }

    void disconnect() override
    {
        disconnect_was_called = true;
    }

    // -----------------------------------------------------------------------
    // Test helper: push a synthetic frame into the registered callback.
    // -----------------------------------------------------------------------
    void inject_frame(const FrameSample & frame)
    {
        if (frame_cb_) {
            ++frames_injected;
            frame_cb_(frame);
        }
    }

    /// Convenience: build and inject a single-body tracking frame.
    void inject_body(int32_t id, float x, float y, float z,
                     float qx = 0.f, float qy = 0.f,
                     float qz = 0.f, float qw = 1.f,
                     int16_t rb_params = 0x01 /* tracking valid */,
                     int16_t frame_params = 0x00)
    {
        FrameSample f;
        f.params = frame_params;
        RigidBodySample rb;
        rb.id = id; rb.x = x; rb.y = y; rb.z = z;
        rb.qx = qx; rb.qy = qy; rb.qz = qz; rb.qw = qw;
        rb.params = rb_params;
        f.bodies.push_back(rb);
        inject_frame(f);
    }

    // -----------------------------------------------------------------------
    // Reset all recorded state (keep configuration).
    // -----------------------------------------------------------------------
    void reset_records()
    {
        connect_was_called      = false;
        server_info_was_called  = false;
        descriptors_was_called  = false;
        set_callback_was_called = false;
        disconnect_was_called   = false;
        frames_injected         = 0;
        last_connect_config     = {};
    }

private:
    std::function<void(const FrameSample &)> frame_cb_;
};

}  // namespace natnet_ros2

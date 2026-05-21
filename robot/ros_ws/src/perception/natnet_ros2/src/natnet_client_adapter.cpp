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
// natnet_client_adapter.cpp — NatNetClientAdapter implementation.
//
// This is the ONLY translation unit that includes NatNet SDK headers.
// All other code (including tests) depends only on INatNetClient.

#include "natnet_ros2/natnet_client_adapter.hpp"

// NatNet SDK (bundled: include/natnet/, lib/libNatNet.so)
#include "NatNetClient.h"
#include "NatNetCAPI.h"
#include "NatNetTypes.h"

#include <cstring>

namespace natnet_ros2
{

// ---------------------------------------------------------------------------
// SDK frame callback trampoline — file-scope so it has C linkage compatible
// with the NATNET_CALLCONV calling convention.
// ---------------------------------------------------------------------------
namespace
{

// We store the user callback per-adapter instance in a thread-local to avoid
// a global.  Limitation: only one adapter instance per thread (sufficient for
// the single-node use case).
thread_local std::function<void(const FrameSample &)> * tl_frame_cb = nullptr;

void NATNET_CALLCONV sdk_frame_callback(sFrameOfMocapData * data, void * /*ctx*/)
{
    if (!data || !tl_frame_cb || !*tl_frame_cb) { return; }

    FrameSample fs;
    fs.frame_num = data->iFrame;
    fs.timestamp = data->fTimestamp;
    fs.params    = static_cast<int16_t>(data->params);

    fs.bodies.reserve(static_cast<std::size_t>(data->nRigidBodies));
    for (int i = 0; i < data->nRigidBodies; ++i) {
        const sRigidBodyData & rb = data->RigidBodies[i];
        RigidBodySample s;
        s.id     = rb.ID;
        s.x      = rb.x;   s.y  = rb.y;  s.z  = rb.z;
        s.qx     = rb.qx;  s.qy = rb.qy; s.qz = rb.qz; s.qw = rb.qw;
        s.params = static_cast<int16_t>(rb.params);
        fs.bodies.push_back(s);
    }

    (*tl_frame_cb)(fs);
}

/// Map NatNet SDK ErrorCode to our NatNetResult.
/// Note: ErrorCode_Timeout was added in NatNet SDK >= 4.5 and is absent in 4.4.
/// If upgrading the SDK, add: case ErrorCode_Timeout: return NatNetResult::Timeout;
NatNetResult from_sdk_error(ErrorCode ec)
{
    switch (ec) {
        case ErrorCode_OK:              return NatNetResult::OK;
        case ErrorCode_Network:         return NatNetResult::NetworkError;
        case ErrorCode_InvalidArgument: return NatNetResult::InvalidAddress;
        default:                        return NatNetResult::InternalError;
    }
}

}  // anonymous namespace


// ---------------------------------------------------------------------------
NatNetClientAdapter::NatNetClientAdapter()
: client_(std::make_unique<NatNetClient>())
{}

NatNetClientAdapter::~NatNetClientAdapter()
{
    disconnect();
}

// ---------------------------------------------------------------------------
NatNetResult NatNetClientAdapter::connect(const ConnectConfig & cfg)
{
    sNatNetClientConnectParams params;
    params.serverAddress     = cfg.server_ip.c_str();
    params.localAddress      = cfg.client_ip.c_str();
    params.serverCommandPort = cfg.command_port;
    params.serverDataPort    = cfg.data_port;

    if (is_multicast(cfg)) {
        params.connectionType   = ConnectionType_Multicast;
        params.multicastAddress = cfg.multicast_address.c_str();
    } else {
        params.connectionType   = ConnectionType_Unicast;
        params.multicastAddress = nullptr;
    }

    return from_sdk_error(client_->Connect(params));
}

// ---------------------------------------------------------------------------
bool NatNetClientAdapter::get_server_info(ServerInfo & out)
{
    sServerDescription desc;
    std::memset(&desc, 0, sizeof(desc));
    const ErrorCode ec = client_->GetServerDescription(&desc);
    if (ec != ErrorCode_OK) { return false; }

    out.host_present    = desc.HostPresent;
    out.host_app_name   = desc.szHostApp;
    for (int i = 0; i < 4; ++i) {
        out.host_app_version[i] = static_cast<int>(desc.HostAppVersion[i]);
        out.natnet_version[i]   = static_cast<int>(desc.NatNetVersion[i]);
    }
    return true;
}

// ---------------------------------------------------------------------------
std::vector<BodyDescriptor> NatNetClientAdapter::get_body_descriptors()
{
    std::vector<BodyDescriptor> result;

    sDataDescriptions * desc_list = nullptr;
    if (client_->GetDataDescriptionList(&desc_list) != ErrorCode_OK || !desc_list) {
        return result;
    }

    for (int i = 0; i < desc_list->nDataDescriptions; ++i) {
        const sDataDescription & dd = desc_list->arrDataDescriptions[i];
        if (dd.type != Descriptor_RigidBody || !dd.Data.RigidBodyDescription) { continue; }
        const sRigidBodyDescription & rb = *dd.Data.RigidBodyDescription;

        BodyDescriptor bd;
        bd.id        = rb.ID;
        bd.name      = rb.szName;
        bd.parent_id = rb.parentID;
        result.push_back(bd);
    }

    NatNet_FreeDescriptions(desc_list);
    return result;
}

// ---------------------------------------------------------------------------
void NatNetClientAdapter::set_frame_callback(
    std::function<void(const FrameSample &)> cb)
{
    user_cb_     = std::move(cb);
    tl_frame_cb  = &user_cb_;
    client_->SetFrameReceivedCallback(sdk_frame_callback, nullptr);
}

// ---------------------------------------------------------------------------
void NatNetClientAdapter::disconnect()
{
    if (client_) {
        client_->Disconnect();
    }
    tl_frame_cb = nullptr;
}

}  // namespace natnet_ros2

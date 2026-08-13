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

// natnet_logic.hpp — pure C++ helpers for natnet_ros2 (no ROS, no NatNet SDK), so
// test_natnet_logic.cpp compiles with only gtest. SDK types stay in
// natnet_ros2_node.cpp / natnet_client_adapter.cpp.

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace natnet_ros2
{

// ===========================================================================
// 1. Covariance
// ===========================================================================

/// Build a row-major 36-element 6×6 covariance from two flat 3×3 blocks.
///
/// pos_cov (up to 9 elements) fills the top-left  3×3 block (rows/cols 0-2).
/// ori_cov (up to 9 elements) fills the bottom-right 3×3 block (rows/cols 3-5).
/// All other entries are zero.
inline std::array<double, 36> build_covariance_6x6(
    const std::vector<double> & pos_cov,
    const std::vector<double> & ori_cov)
{
    std::array<double, 36> cov{};
    cov.fill(0.0);
    const int np = static_cast<int>(pos_cov.size());
    const int no = static_cast<int>(ori_cov.size());
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const int idx = r * 3 + c;
            if (idx < np) { cov[r * 6 + c] = pos_cov[idx]; }
            if (idx < no) { cov[(r + 3) * 6 + (c + 3)] = ori_cov[idx]; }
        }
    }
    return cov;
}


// ===========================================================================
// 2. Topic names
// ===========================================================================

/// Base topic for a rigid body: /{robot_name}/perception/optitrack/{body_name}
inline std::string optitrack_topic_base(
    const std::string & robot_name,
    const std::string & body_name)
{
    return "/" + robot_name + "/perception/optitrack/" + body_name;
}

/// PoseWithCovarianceStamped topic: …/{body_name}/pose_cov
inline std::string optitrack_pose_cov_topic(
    const std::string & robot_name,
    const std::string & body_name)
{
    return optitrack_topic_base(robot_name, body_name) + "/pose_cov";
}

/// Namespace a relative topic leaf under /{robot_name}/.
///
/// Leading slashes in \p relative are stripped so the result always has exactly
/// one. Used for the per-body ``topic`` overrides in natnet_config.yaml, which are
/// relative and namespaced by the node at runtime.
inline std::string namespaced_topic(
    const std::string & robot_name,
    const std::string & relative)
{
    const std::size_t start = relative.find_first_not_of('/');
    const std::string leaf =
        (start == std::string::npos) ? std::string{} : relative.substr(start);
    return "/" + robot_name + "/" + leaf;
}

/// Topic base for one configured body: the per-body relative override when set,
/// otherwise the default /{robot_name}/perception/optitrack/{body_name}.
inline std::string body_topic_base(
    const std::string & robot_name,
    const std::string & body_name,
    const std::string & relative_override)
{
    if (relative_override.empty()) {
        return optitrack_topic_base(robot_name, body_name);
    }
    return namespaced_topic(robot_name, relative_override);
}


// ===========================================================================
// 3. Connection-configuration helpers
// ===========================================================================

/// Return ct if it is "unicast" or "multicast"; otherwise return "unicast".
inline std::string validate_connection_type(const std::string & ct)
{
    if (ct == "unicast" || ct == "multicast") { return ct; }
    return "unicast";
}

/// SDK-independent connection configuration aggregate.
///
/// natnet_ros2_node.cpp converts this into sNatNetClientConnectParams; tests
/// exercise the pure logic without linking the NatNet SDK.
struct ConnectConfig
{
    std::string server_ip         = "192.168.1.1";
    std::string client_ip         = "0.0.0.0";
    uint16_t    command_port      = 1510u;
    uint16_t    data_port         = 1511u;
    std::string connection_type   = "unicast";      ///< validated
    std::string multicast_address = "239.255.42.99";
};

/// Build a validated ConnectConfig from raw user-supplied strings.
/// connection_type is normalised via validate_connection_type().
inline ConnectConfig make_connect_config(
    const std::string & server_ip,
    const std::string & client_ip,
    uint16_t command_port,
    uint16_t data_port,
    const std::string & connection_type,
    const std::string & multicast_address = "239.255.42.99")
{
    return ConnectConfig{
        server_ip,
        client_ip,
        command_port,
        data_port,
        validate_connection_type(connection_type),
        multicast_address
    };
}

/// Returns true when the config requests a multicast connection.
inline bool is_multicast(const ConnectConfig & cfg)
{
    return cfg.connection_type == "multicast";
}

/// Returns true when the multicast address should be used.
/// When false, the multicast_address field is irrelevant and should be nullptr
/// when passed to sNatNetClientConnectParams.
inline bool needs_multicast_address(const ConnectConfig & cfg)
{
    return is_multicast(cfg);
}


// ===========================================================================
// 4. Rigid-body frame helpers (SDK-independent)
// ===========================================================================

/// Lightweight, SDK-free representation of a single rigid-body sample.
/// natnet_ros2_node.cpp converts sRigidBodyData → RigidBodySample.
struct RigidBodySample
{
    int32_t id     = 0;
    float   x      = 0.f;
    float   y      = 0.f;
    float   z      = 0.f;
    float   qx     = 0.f;
    float   qy     = 0.f;
    float   qz     = 0.f;
    float   qw     = 1.f;
    int16_t params = 0;   ///< NatNet rb.params bitmask
};

/// Lightweight, SDK-free representation of one frame of mocap data.
struct FrameSample
{
    int32_t frame_num = 0;
    float   timestamp = 0.f;
    int16_t params    = 0;   ///< NatNet frame.params bitmask
    /// Seconds elapsed since the server transmitted this frame, as reported by
    /// NatNetClient::SecondsSinceHostTimestamp(TransmitTimestamp). This is the
    /// transit + client-processing latency the drone observes per message.
    double  transit_latency_s = 0.0;
    /// True when transit_latency_s is meaningful (server supplied a non-zero
    /// TransmitTimestamp). Older servers / streams without timing info leave it false.
    bool    has_latency = false;
    std::vector<RigidBodySample> bodies;
};

/// Returns true when bit 0 of rb.params is set (NatNet: tracking valid).
inline bool is_tracking_valid(int16_t rb_params)
{
    return (rb_params & 0x01) != 0;
}

/// Returns true when bit 1 of frame.params is set (NatNet: model list changed).
inline bool model_list_changed(int16_t frame_params)
{
    return (frame_params & 0x02) != 0;
}

/// Returns true when the rigid body should be published.
/// filter_id < 0 means "publish all bodies"; otherwise only the matching ID.
inline bool should_publish_body(int32_t filter_id, int32_t rb_id)
{
    return filter_id < 0 || rb_id == filter_id;
}

/// Returns true when rb_id is one of the configured body ids.
///
/// Multi-body variant of should_publish_body(): the node tracks a fixed set of
/// ids from natnet_config.yaml and publishes only those (empty set → nothing).
inline bool body_is_configured(const std::vector<int32_t> & configured_ids, int32_t rb_id)
{
    return std::find(configured_ids.begin(), configured_ids.end(), rb_id)
           != configured_ids.end();
}

/// Double-precision pose extracted from a RigidBodySample.
struct PoseData
{
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

/// Convert a RigidBodySample to a double-precision PoseData.
inline PoseData rb_to_pose(const RigidBodySample & rb)
{
    return PoseData{
        static_cast<double>(rb.x),
        static_cast<double>(rb.y),
        static_cast<double>(rb.z),
        static_cast<double>(rb.qx),
        static_cast<double>(rb.qy),
        static_cast<double>(rb.qz),
        static_cast<double>(rb.qw)
    };
}

/// Fill a 36-element covariance array into a pre-allocated ROS-style covariance
/// field from a pre-built std::array<double,36>.
/// Returns a copy of the array (ROS msg.covariance = cov6x6_to_array(...)).
inline std::array<double, 36> cov6x6_to_array(const std::array<double, 36> & src)
{
    return src;
}

// ===========================================================================
// 5. Abstraction seam: INatNetClient + negotiation logic
// ===========================================================================

/// SDK-independent result codes for connection attempts.
enum class NatNetResult
{
    OK,
    NetworkError,
    InvalidAddress,
    Timeout,
    InternalError,
};

inline const char * natnet_result_str(NatNetResult r)
{
    switch (r) {
        case NatNetResult::OK:             return "OK";
        case NatNetResult::NetworkError:   return "NetworkError";
        case NatNetResult::InvalidAddress: return "InvalidAddress";
        case NatNetResult::Timeout:        return "Timeout";
        case NatNetResult::InternalError:  return "InternalError";
    }
    return "Unknown";
}

/// Server identity returned after a successful connection.
struct ServerInfo
{
    bool        host_present        = false;
    std::string host_app_name;
    int         host_app_version[4] = {};  ///< major.minor.build.revision
    int         natnet_version[4]   = {};  ///< major.minor.build.revision
};

/// SDK-independent description of one rigid-body asset.
struct BodyDescriptor
{
    int32_t     id        = 0;
    std::string name;
    int32_t     parent_id = -1;  ///< >= 0 → skeleton bone; skip for top-level publishing
};

/// Result of the connect + GetServerDescription handshake.
struct NegotiationResult
{
    bool         ok           = false;
    ServerInfo   server_info;
    std::string  log_message;  ///< human-readable outcome for the ROS logger
};

/// Pure-virtual client interface — implemented by NatNetClientAdapter (production)
/// and FakeNatNetClient (unit tests).
///
/// Depends only on natnet_logic.hpp types; never includes NatNet SDK headers.
class INatNetClient
{
public:
    virtual ~INatNetClient() = default;

    /// Attempt to connect to a Motive server.
    virtual NatNetResult connect(const ConnectConfig & cfg) = 0;

    /// Populate \p out with server identity.  Returns false when host info is
    /// unavailable (HostPresent == false in the SDK's sServerDescription).
    virtual bool get_server_info(ServerInfo & out) = 0;

    /// Return descriptions of all rigid-body assets currently known to Motive.
    /// Returns empty on failure; callers should retry on model-list-changed.
    virtual std::vector<BodyDescriptor> get_body_descriptors() = 0;

    /// Register a callback invoked on every incoming frame.
    /// The callback is called from the SDK receive thread.
    virtual void set_frame_callback(std::function<void(const FrameSample &)> cb) = 0;

    /// Disconnect from the server and release SDK resources.
    virtual void disconnect() = 0;
};

/// Execute the connect + GetServerDescription handshake and return a structured
/// result.  Pure logic: no ROS calls, no SDK types — fully testable with a fake.
inline NegotiationResult negotiate(INatNetClient & client, const ConnectConfig & cfg)
{
    NegotiationResult result;

    const NatNetResult err = client.connect(cfg);
    if (err != NatNetResult::OK) {
        result.ok          = false;
        result.log_message = std::string("NatNetClient::Connect failed (")
                           + natnet_result_str(err)
                           + ") — server=" + cfg.server_ip
                           + " port=" + std::to_string(cfg.command_port)
                           + " type=" + cfg.connection_type;
        return result;
    }

    result.ok = true;

    const bool host_ok = client.get_server_info(result.server_info);
    if (!host_ok || !result.server_info.host_present) {
        result.log_message = "Connected to " + cfg.server_ip
                           + " but GetServerDescription returned no host info.";
    } else {
        result.log_message = "Connected to Motive '"
                           + result.server_info.host_app_name
                           + "' v"
                           + std::to_string(result.server_info.host_app_version[0])
                           + "."
                           + std::to_string(result.server_info.host_app_version[1])
                           + " (NatNet "
                           + std::to_string(result.server_info.natnet_version[0])
                           + "."
                           + std::to_string(result.server_info.natnet_version[1])
                           + ") at " + cfg.server_ip;
    }
    return result;
}

}  // namespace natnet_ros2

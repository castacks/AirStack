// natnet_logic.hpp — pure C++ helpers for natnet_ros2 (no ROS, no NatNet SDK).
//
// Four responsibility areas:
//
//  1. Covariance assembly
//  2. Topic names
//  3. Connection-configuration helpers (SDK-independent)
//  4. Rigid-body frame helpers (SDK-independent, for unit-testing frame logic)
//
// The NatNet SDK types (sNatNetClientConnectParams, sRigidBodyData, …) are only
// used inside natnet_ros2_node.cpp.  All logic operated on here uses plain C++
// types so that test/test_natnet_logic.cpp compiles with only gtest.

#pragma once

#include <array>
#include <cstdint>
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

}  // namespace natnet_ros2

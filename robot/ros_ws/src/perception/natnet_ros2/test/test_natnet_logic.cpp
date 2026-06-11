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
// Unit tests for natnet_ros2/natnet_logic.hpp.
//
// NO dependency on the NatNet SDK or rclcpp — compiles with gtest only.
// Run via:
//   colcon test --packages-select natnet_ros2 --event-handlers console_direct+
//   colcon test-result --test-result-base build/natnet_ros2 --verbose

#include <gtest/gtest.h>
#include "natnet_ros2/natnet_logic.hpp"
#include "fake_natnet_client.hpp"

using namespace natnet_ros2;


// ===========================================================================
// Covariance
// ===========================================================================

TEST(BuildCovariance6x6, DiagonalBlocksLandInCorrectSlots)
{
    const std::vector<double> pos = {0.1, 0.0, 0.0,
                                     0.0, 0.1, 0.0,
                                     0.0, 0.0, 0.1};
    const std::vector<double> ori = {0.01, 0.0, 0.0,
                                     0.0, 0.01, 0.0,
                                     0.0, 0.0, 0.01};

    auto cov = build_covariance_6x6(pos, ori);

    ASSERT_EQ(cov.size(), 36u);
    EXPECT_DOUBLE_EQ(cov[0 * 6 + 0], 0.1);
    EXPECT_DOUBLE_EQ(cov[1 * 6 + 1], 0.1);
    EXPECT_DOUBLE_EQ(cov[2 * 6 + 2], 0.1);
    EXPECT_DOUBLE_EQ(cov[3 * 6 + 3], 0.01);
    EXPECT_DOUBLE_EQ(cov[4 * 6 + 4], 0.01);
    EXPECT_DOUBLE_EQ(cov[5 * 6 + 5], 0.01);
}

TEST(BuildCovariance6x6, CrossBlockEntriesAreZero)
{
    const std::vector<double> ones(9, 1.0);
    auto cov = build_covariance_6x6(ones, ones);

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            const bool in_pos = (r < 3 && c < 3);
            const bool in_ori = (r >= 3 && c >= 3);
            if (!in_pos && !in_ori) {
                EXPECT_DOUBLE_EQ(cov[r * 6 + c], 0.0)
                    << "Expected 0 at [" << r << "][" << c << "]";
            }
        }
    }
}

TEST(BuildCovariance6x6, OffDiagonalEntriesPreserved)
{
    const std::vector<double> pos = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    const std::vector<double> ori = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};

    auto cov = build_covariance_6x6(pos, ori);

    EXPECT_DOUBLE_EQ(cov[1 * 6 + 2], 6.0);   // pos[1][2]
    EXPECT_DOUBLE_EQ(cov[2 * 6 + 0], 7.0);   // pos[2][0]
    EXPECT_DOUBLE_EQ(cov[3 * 6 + 4], 0.2);   // ori[0][1]
    EXPECT_DOUBLE_EQ(cov[5 * 6 + 4], 0.8);   // ori[2][1]
}

TEST(BuildCovariance6x6, ShortInputFillsRemainingWithZero)
{
    auto cov = build_covariance_6x6({0.5}, {0.05});

    EXPECT_DOUBLE_EQ(cov[0 * 6 + 0], 0.5);
    EXPECT_DOUBLE_EQ(cov[3 * 6 + 3], 0.05);
    EXPECT_DOUBLE_EQ(cov[0 * 6 + 1], 0.0);
    EXPECT_DOUBLE_EQ(cov[4 * 6 + 4], 0.0);
}

TEST(BuildCovariance6x6, EmptyInputsProduceAllZeros)
{
    auto cov = build_covariance_6x6({}, {});
    for (double v : cov) { EXPECT_DOUBLE_EQ(v, 0.0); }
}

TEST(BuildCovariance6x6, OutputIsExactly36Elements)
{
    EXPECT_EQ(static_cast<int>(build_covariance_6x6({}, {}).size()), 36);
}


// ===========================================================================
// Topic names
// ===========================================================================

TEST(TopicNames, BaseTopicFormat)
{
    EXPECT_EQ(optitrack_topic_base("robot_1", "Drone"),
              "/robot_1/perception/optitrack/Drone");
}

TEST(TopicNames, PoseCovTopicAppendsSuffix)
{
    const std::string base = optitrack_topic_base("robot_1", "Drone");
    const std::string cov  = optitrack_pose_cov_topic("robot_1", "Drone");
    EXPECT_EQ(cov, base + "/pose_cov");
}

TEST(TopicNames, DifferentRobotsGetDifferentNamespaces)
{
    EXPECT_NE(optitrack_topic_base("robot_1", "Body"),
              optitrack_topic_base("robot_2", "Body"));
}

TEST(TopicNames, LeadingSlashPresent)
{
    EXPECT_EQ(optitrack_topic_base("robot_1", "Body")[0], '/');
}


// ===========================================================================
// Server negotiation — validate_connection_type
// ===========================================================================

TEST(ValidateConnectionType, UnicastPassesThrough)
{
    EXPECT_EQ(validate_connection_type("unicast"), "unicast");
}

TEST(ValidateConnectionType, MulticastPassesThrough)
{
    EXPECT_EQ(validate_connection_type("multicast"), "multicast");
}

TEST(ValidateConnectionType, UnknownFallsBackToUnicast)
{
    EXPECT_EQ(validate_connection_type("broadcast"), "unicast");
    EXPECT_EQ(validate_connection_type(""), "unicast");
    EXPECT_EQ(validate_connection_type("UDP"), "unicast");
}

TEST(ValidateConnectionType, CaseSensitiveFallsBack)
{
    EXPECT_EQ(validate_connection_type("Unicast"),   "unicast");
    EXPECT_EQ(validate_connection_type("MULTICAST"), "unicast");
}


// ===========================================================================
// Server negotiation — ConnectConfig + make_connect_config
// ===========================================================================

TEST(ConnectConfig, DefaultsAreUnicast)
{
    const ConnectConfig cfg{};
    EXPECT_EQ(cfg.connection_type, "unicast");
    EXPECT_FALSE(is_multicast(cfg));
}

TEST(ConnectConfig, UnicastConfigNotMulticast)
{
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510, 1511, "unicast");
    EXPECT_FALSE(is_multicast(cfg));
    EXPECT_FALSE(needs_multicast_address(cfg));
}

TEST(ConnectConfig, MulticastConfigIsMulticast)
{
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510, 1511, "multicast", "239.255.42.99");
    EXPECT_TRUE(is_multicast(cfg));
    EXPECT_TRUE(needs_multicast_address(cfg));
    EXPECT_EQ(cfg.multicast_address, "239.255.42.99");
}

TEST(ConnectConfig, InvalidConnectionTypeFallsBackToUnicast)
{
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510, 1511, "broadcast");
    EXPECT_EQ(cfg.connection_type, "unicast");
    EXPECT_FALSE(is_multicast(cfg));
}

TEST(ConnectConfig, PortsArePreserved)
{
    const auto cfg = make_connect_config(
        "192.168.0.100", "192.168.0.200", 9000u, 9001u, "unicast");
    EXPECT_EQ(cfg.server_ip,    "192.168.0.100");
    EXPECT_EQ(cfg.client_ip,    "192.168.0.200");
    EXPECT_EQ(cfg.command_port, 9000u);
    EXPECT_EQ(cfg.data_port,    9001u);
}

TEST(ConnectConfig, CustomMulticastAddress)
{
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510, 1511, "multicast", "239.0.0.1");
    EXPECT_EQ(cfg.multicast_address, "239.0.0.1");
}

TEST(ConnectConfig, UnicastAddressFieldIgnored)
{
    // multicast_address is still stored but should not be passed to the SDK
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510, 1511, "unicast", "239.255.42.99");
    EXPECT_FALSE(needs_multicast_address(cfg));
}


// ===========================================================================
// Data streaming — is_tracking_valid
// ===========================================================================

TEST(IsTrackingValid, Bit0SetMeansValid)
{
    EXPECT_TRUE(is_tracking_valid(0x01));
    EXPECT_TRUE(is_tracking_valid(0x03));  // bits 0 and 1
    EXPECT_TRUE(is_tracking_valid(0xFF));
}

TEST(IsTrackingValid, Bit0ClearMeansInvalid)
{
    EXPECT_FALSE(is_tracking_valid(0x00));
    EXPECT_FALSE(is_tracking_valid(0x02));  // only bit 1 set
    EXPECT_FALSE(is_tracking_valid(0xFE));  // all bits except 0
}


// ===========================================================================
// Data streaming — model_list_changed
// ===========================================================================

TEST(ModelListChanged, Bit1SetMeansChanged)
{
    EXPECT_TRUE(model_list_changed(0x02));
    EXPECT_TRUE(model_list_changed(0x03));
    EXPECT_TRUE(model_list_changed(0xFF));
}

TEST(ModelListChanged, Bit1ClearMeansNotChanged)
{
    EXPECT_FALSE(model_list_changed(0x00));
    EXPECT_FALSE(model_list_changed(0x01));  // only bit 0
    EXPECT_FALSE(model_list_changed(0xFD));  // all bits except 1
}


// ===========================================================================
// Data streaming — should_publish_body
// ===========================================================================

TEST(ShouldPublishBody, NegativeFilterMeansPublishAll)
{
    EXPECT_TRUE(should_publish_body(-1, 0));
    EXPECT_TRUE(should_publish_body(-1, 1));
    EXPECT_TRUE(should_publish_body(-1, 999));
}

TEST(ShouldPublishBody, ZeroFilterAllowsOnlyId0)
{
    EXPECT_TRUE(should_publish_body(0, 0));
    EXPECT_FALSE(should_publish_body(0, 1));
    EXPECT_FALSE(should_publish_body(0, 999));
}

TEST(ShouldPublishBody, PositiveFilterMatchesExact)
{
    EXPECT_TRUE(should_publish_body(5, 5));
    EXPECT_FALSE(should_publish_body(5, 4));
    EXPECT_FALSE(should_publish_body(5, 6));
}


// ===========================================================================
// Data streaming — rb_to_pose (sample data conversion)
// ===========================================================================

TEST(RbToPose, PositionComponentsConvertedToDouble)
{
    RigidBodySample rb;
    rb.x = 1.5f;  rb.y = -2.25f;  rb.z = 0.5f;
    rb.qx = 0.f;  rb.qy = 0.f;    rb.qz = 0.f;  rb.qw = 1.f;

    const PoseData p = rb_to_pose(rb);

    EXPECT_DOUBLE_EQ(p.x, static_cast<double>(1.5f));
    EXPECT_DOUBLE_EQ(p.y, static_cast<double>(-2.25f));
    EXPECT_DOUBLE_EQ(p.z, static_cast<double>(0.5f));
}

TEST(RbToPose, OrientationComponentsConvertedToDouble)
{
    RigidBodySample rb;
    rb.x = 0.f; rb.y = 0.f; rb.z = 0.f;
    // 90-degree rotation about Z: qw = cos(45°), qz = sin(45°)
    rb.qx = 0.f;
    rb.qy = 0.f;
    rb.qz = 0.7071068f;
    rb.qw = 0.7071068f;

    const PoseData p = rb_to_pose(rb);

    EXPECT_NEAR(p.qz, 0.7071068, 1e-6);
    EXPECT_NEAR(p.qw, 0.7071068, 1e-6);
    EXPECT_DOUBLE_EQ(p.qx, 0.0);
    EXPECT_DOUBLE_EQ(p.qy, 0.0);
}

TEST(RbToPose, IdentityOrientationPreserved)
{
    RigidBodySample rb;  // default: x=y=z=0, qw=1
    const PoseData p = rb_to_pose(rb);

    EXPECT_DOUBLE_EQ(p.x,  0.0);
    EXPECT_DOUBLE_EQ(p.y,  0.0);
    EXPECT_DOUBLE_EQ(p.z,  0.0);
    EXPECT_DOUBLE_EQ(p.qx, 0.0);
    EXPECT_DOUBLE_EQ(p.qy, 0.0);
    EXPECT_DOUBLE_EQ(p.qz, 0.0);
    EXPECT_DOUBLE_EQ(p.qw, 1.0);
}

TEST(RbToPose, NegativeCoordinates)
{
    RigidBodySample rb;
    rb.x = -10.f; rb.y = -20.f; rb.z = -30.f;
    rb.qw = 1.f;

    const PoseData p = rb_to_pose(rb);

    EXPECT_DOUBLE_EQ(p.x, static_cast<double>(-10.f));
    EXPECT_DOUBLE_EQ(p.y, static_cast<double>(-20.f));
    EXPECT_DOUBLE_EQ(p.z, static_cast<double>(-30.f));
}


// ===========================================================================
// Data streaming — FrameSample helpers (integration-style scenarios)
// ===========================================================================

// Simulate a frame where one body is tracking and one is not.
TEST(FrameSample, TrackingFilterApplied)
{
    FrameSample frame;
    frame.frame_num = 42;
    frame.timestamp = 1.234f;
    frame.params    = 0x00;

    RigidBodySample tracking, lost;
    tracking.id     = 1;
    tracking.params = 0x01;   // valid
    lost.id         = 2;
    lost.params     = 0x00;   // invalid

    frame.bodies = {tracking, lost};

    int published = 0;
    for (const auto & rb : frame.bodies) {
        if (is_tracking_valid(rb.params)) {
            ++published;
        }
    }
    EXPECT_EQ(published, 1);
}

// Simulate a frame that signals model-list changed while also carrying data.
TEST(FrameSample, ModelListChangedFlagDetected)
{
    FrameSample frame;
    frame.params = 0x02;  // bit 1 set

    EXPECT_TRUE(model_list_changed(frame.params));
}

// Simulate single-body tracking filter: only body id=3 should be published.
TEST(FrameSample, SingleBodyFilterSelectsCorrectBody)
{
    FrameSample frame;
    frame.params = 0x00;

    for (int id : {1, 2, 3, 4, 5}) {
        RigidBodySample rb;
        rb.id     = id;
        rb.params = 0x01;   // all tracking valid
        frame.bodies.push_back(rb);
    }

    constexpr int32_t filter = 3;
    std::vector<int32_t> published;

    for (const auto & rb : frame.bodies) {
        if (is_tracking_valid(rb.params) && should_publish_body(filter, rb.id)) {
            published.push_back(rb.id);
        }
    }

    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0], 3);
}

// Simulate all-body mode: every valid body gets a PoseData.
TEST(FrameSample, AllBodyModePublishesAllTrackedBodies)
{
    FrameSample frame;
    for (int id = 1; id <= 4; ++id) {
        RigidBodySample rb;
        rb.id     = id;
        rb.x      = static_cast<float>(id);
        rb.params = (id % 2 == 0) ? int16_t(0x01) : int16_t(0x00);  // even = valid
        frame.bodies.push_back(rb);
    }

    std::vector<PoseData> out;
    for (const auto & rb : frame.bodies) {
        if (is_tracking_valid(rb.params) && should_publish_body(-1, rb.id)) {
            out.push_back(rb_to_pose(rb));
        }
    }

    // Bodies 2 and 4 are valid
    ASSERT_EQ(out.size(), 2u);
    EXPECT_DOUBLE_EQ(out[0].x, static_cast<double>(2.f));
    EXPECT_DOUBLE_EQ(out[1].x, static_cast<double>(4.f));
}

// Verify covariance is stamped into the output as expected.
TEST(FrameSample, CovarianceStampedIntoMessage)
{
    const std::vector<double> pos_cov(9, 0.1);
    const std::vector<double> ori_cov(9, 0.01);
    const auto cov = build_covariance_6x6(pos_cov, ori_cov);

    // Simulate what natnet_ros2_node.cpp does when building PoseWithCovarianceStamped
    std::array<double, 36> msg_covariance = cov6x6_to_array(cov);

    // Position diagonal
    EXPECT_DOUBLE_EQ(msg_covariance[0 * 6 + 0], 0.1);
    EXPECT_DOUBLE_EQ(msg_covariance[1 * 6 + 1], 0.1);
    EXPECT_DOUBLE_EQ(msg_covariance[2 * 6 + 2], 0.1);
    // Orientation diagonal
    EXPECT_DOUBLE_EQ(msg_covariance[3 * 6 + 3], 0.01);
    EXPECT_DOUBLE_EQ(msg_covariance[4 * 6 + 4], 0.01);
    EXPECT_DOUBLE_EQ(msg_covariance[5 * 6 + 5], 0.01);
    // Cross-block zeros
    EXPECT_DOUBLE_EQ(msg_covariance[0 * 6 + 3], 0.0);
    EXPECT_DOUBLE_EQ(msg_covariance[3 * 6 + 0], 0.0);
}


// ===========================================================================
// Server negotiation — negotiate() + FakeNatNetClient
// ===========================================================================

// --------------- helpers ---------------------------------------------------

static ConnectConfig make_test_cfg(const std::string & ct = "unicast")
{
    return make_connect_config("192.168.1.100", "0.0.0.0", 1510u, 1511u, ct);
}

static ServerInfo make_server_info(bool present = true,
                                   const std::string & app = "Motive",
                                   int vmaj = 3, int vmin = 1,
                                   int nnmaj = 4, int nnmin = 1)
{
    ServerInfo si;
    si.host_present         = present;
    si.host_app_name        = app;
    si.host_app_version[0]  = vmaj;
    si.host_app_version[1]  = vmin;
    si.natnet_version[0]    = nnmaj;
    si.natnet_version[1]    = nnmin;
    return si;
}

// ----------- negotiate() success paths ------------------------------------

TEST(Negotiate, SuccessWithHostPresent)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::OK;
    fake.server_info    = make_server_info(true, "Motive", 3, 1, 4, 1);

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.server_info.host_present);
    EXPECT_EQ(result.server_info.host_app_name, "Motive");
    EXPECT_EQ(result.server_info.host_app_version[0], 3);
    EXPECT_EQ(result.server_info.natnet_version[0], 4);

    EXPECT_TRUE(fake.connect_was_called);
    EXPECT_TRUE(fake.server_info_was_called);
    // log message should mention the server IP
    EXPECT_NE(result.log_message.find("192.168.1.100"), std::string::npos);
}

TEST(Negotiate, SuccessButHostNotPresent)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::OK;
    fake.server_info    = make_server_info(false);

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_TRUE(result.ok);             // connection itself succeeded
    EXPECT_FALSE(result.server_info.host_present);
    // log message should flag the missing host info
    EXPECT_NE(result.log_message.find("no host info"), std::string::npos);
}

TEST(Negotiate, SuccessServerInfoCallFails)
{
    // SDK's GetServerDescription returns an error (simulated via call_succeeds=false)
    FakeNatNetClient fake;
    fake.connect_result              = NatNetResult::OK;
    fake.server_info_call_succeeds   = false;

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(result.server_info.host_present);
    EXPECT_NE(result.log_message.find("no host info"), std::string::npos);
}

// ----------- negotiate() failure paths ------------------------------------

TEST(Negotiate, NetworkErrorReturnsFalse)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::NetworkError;

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(fake.server_info_was_called);  // should not reach GetServerDescription
    EXPECT_NE(result.log_message.find("NetworkError"), std::string::npos);
    EXPECT_NE(result.log_message.find("192.168.1.100"), std::string::npos);
}

TEST(Negotiate, InvalidAddressReturnsFalse)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::InvalidAddress;

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.log_message.find("InvalidAddress"), std::string::npos);
}

TEST(Negotiate, TimeoutReturnsFalse)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::Timeout;

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.log_message.find("Timeout"), std::string::npos);
}

// ----------- negotiate() passes ConnectConfig correctly -------------------

TEST(Negotiate, UnicastConfigPassedToClient)
{
    FakeNatNetClient fake;
    const auto cfg = make_test_cfg("unicast");
    negotiate(fake, cfg);

    EXPECT_EQ(fake.last_connect_config.connection_type, "unicast");
    EXPECT_EQ(fake.last_connect_config.server_ip, "192.168.1.100");
    EXPECT_EQ(fake.last_connect_config.command_port, 1510u);
}

TEST(Negotiate, MulticastConfigPassedToClient)
{
    FakeNatNetClient fake;
    const auto cfg = make_connect_config(
        "10.0.0.1", "0.0.0.0", 1510u, 1511u, "multicast", "239.0.0.1");
    negotiate(fake, cfg);

    EXPECT_EQ(fake.last_connect_config.connection_type, "multicast");
    EXPECT_EQ(fake.last_connect_config.multicast_address, "239.0.0.1");
}

// ----------- log message content ------------------------------------------

TEST(Negotiate, SuccessLogContainsAppAndVersion)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::OK;
    fake.server_info    = make_server_info(true, "MotiveBody", 2, 5, 4, 0);

    const auto result = negotiate(fake, make_test_cfg());

    EXPECT_NE(result.log_message.find("MotiveBody"), std::string::npos);
    EXPECT_NE(result.log_message.find("2.5"),        std::string::npos);  // v2.5
    EXPECT_NE(result.log_message.find("4.0"),        std::string::npos);  // NatNet 4.0
}

TEST(Negotiate, FailureLogContainsPortAndType)
{
    FakeNatNetClient fake;
    fake.connect_result = NatNetResult::Timeout;
    const auto cfg = make_connect_config(
        "10.1.2.3", "0.0.0.0", 9000u, 9001u, "multicast");

    const auto result = negotiate(fake, cfg);

    EXPECT_NE(result.log_message.find("9000"),      std::string::npos);
    EXPECT_NE(result.log_message.find("multicast"), std::string::npos);
}


// ===========================================================================
// FakeNatNetClient — frame injection
// ===========================================================================

TEST(FakeNatNetClient, CallbackNotCalledBeforeRegistration)
{
    FakeNatNetClient fake;
    // No set_frame_callback called — inject_frame should be a no-op
    fake.inject_body(1, 1.f, 2.f, 3.f);
    EXPECT_EQ(fake.frames_injected, 0);
}

TEST(FakeNatNetClient, CallbackInvokedAfterRegistration)
{
    FakeNatNetClient fake;

    std::vector<FrameSample> received;
    fake.set_frame_callback([&](const FrameSample & f) { received.push_back(f); });

    fake.inject_body(1, 1.f, 2.f, 3.f);
    fake.inject_body(2, 4.f, 5.f, 6.f);

    EXPECT_EQ(fake.frames_injected, 2);
    ASSERT_EQ(received.size(), 2u);
}

TEST(FakeNatNetClient, InjectedBodyDataIsPreserved)
{
    FakeNatNetClient fake;

    FrameSample captured;
    fake.set_frame_callback([&](const FrameSample & f) { captured = f; });

    fake.inject_body(42, 1.5f, -2.5f, 0.75f,
                     0.f, 0.f, 0.7071068f, 0.7071068f,
                     0x01 /* tracking valid */);

    ASSERT_EQ(captured.bodies.size(), 1u);
    const auto & rb = captured.bodies[0];
    EXPECT_EQ(rb.id, 42);
    EXPECT_FLOAT_EQ(rb.x,  1.5f);
    EXPECT_FLOAT_EQ(rb.y, -2.5f);
    EXPECT_FLOAT_EQ(rb.z,  0.75f);
    EXPECT_NEAR(rb.qz, 0.7071068f, 1e-6f);
    EXPECT_TRUE(is_tracking_valid(rb.params));
}

TEST(FakeNatNetClient, ModelListChangedFlagDeliveredInFrame)
{
    FakeNatNetClient fake;

    bool model_changed = false;
    fake.set_frame_callback([&](const FrameSample & f) {
        model_changed = model_list_changed(f.params);
    });

    FrameSample f;
    f.params = 0x02;  // bit 1 = model list changed
    fake.inject_frame(f);

    EXPECT_TRUE(model_changed);
}

TEST(FakeNatNetClient, ResetRecordsClearsState)
{
    FakeNatNetClient fake;
    fake.set_frame_callback([](const FrameSample &) {});
    fake.inject_body(1, 0.f, 0.f, 0.f);

    fake.reset_records();

    EXPECT_FALSE(fake.connect_was_called);
    EXPECT_FALSE(fake.set_callback_was_called);
    EXPECT_EQ(fake.frames_injected, 0);
}


// ===========================================================================
// Body descriptors + filtering
// ===========================================================================

TEST(BodyDescriptor, SkeletonBoneHasPositiveParentId)
{
    BodyDescriptor bone;
    bone.id        = 10;
    bone.name      = "Hip";
    bone.parent_id = 5;  // part of skeleton with id=5

    EXPECT_GE(bone.parent_id, 0);  // should be skipped in publisher creation
}

TEST(BodyDescriptor, TopLevelBodyHasNegativeParentId)
{
    BodyDescriptor body;
    body.id        = 1;
    body.name      = "Drone";
    body.parent_id = -1;

    EXPECT_LT(body.parent_id, 0);  // should be published
}

TEST(FakeNatNetClient, GetBodyDescriptorsReturnsConfigured)
{
    FakeNatNetClient fake;
    fake.body_descriptors = {
        {1, "Drone1", -1},
        {2, "Drone2", -1},
        {3, "Hip",     2},   // skeleton bone
    };

    const auto descs = fake.get_body_descriptors();

    ASSERT_EQ(descs.size(), 3u);
    EXPECT_TRUE(fake.descriptors_was_called);

    // Only top-level bodies should be published (parent_id < 0)
    int top_level = 0;
    for (const auto & d : descs) {
        if (d.parent_id < 0) { ++top_level; }
    }
    EXPECT_EQ(top_level, 2);
}

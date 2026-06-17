// natnet_ros2_node.cpp
//
// ROS 2 NatNet SDK node for OptiTrack Motive integration.
//
// Published topics (per configured rigid body):
//   /{robot_name}/{topic}            → PoseStamped               (when pose=true)
//   /{robot_name}/{topic}/pose_cov   → PoseWithCovarianceStamped (when pose_cov=true)
// where {topic} defaults to perception/optitrack/{rigid_body_name} when unset.
//
// Parameters are flattened from config/natnet_config.yaml by natnet_ros2.launch.py:
//   server_ip, client_ip, command_port, data_port, connection_type,
//   multicast_address, frame_id, debug, and parallel per-body arrays:
//   body_names[], body_ids[], body_topics[], body_pose[], body_pose_cov[],
//   body_position_covariance[] / body_orientation_covariance[] (9·N, sliced per body).
//
// ROBOT_NAME is read from the environment variable set by AirStack's
// robot_name_map resolver at container startup.

#include <algorithm>
#include <array>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Pure logic + interface (no SDK, testable with FakeNatNetClient)
#include "natnet_ros2/natnet_logic.hpp"
#include "natnet_ros2/natnet_client_adapter.hpp"


// ---------------------------------------------------------------------------
// NatNetROS2Node
// ---------------------------------------------------------------------------
class NatNetROS2Node : public rclcpp::Node
{
public:
    using PoseStamped               = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

    struct BodyConfig
    {
        int32_t                                                id = -1;
        std::string                                            rigid_body_name;
        std::string                                            topic_base;
        bool                                                   publish_pose     = true;
        bool                                                   publish_pose_cov = true;
        std::array<double, 36>                                 covariance{};
        rclcpp::Publisher<PoseStamped>::SharedPtr               pose_pub;
        rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;
    };

    // -----------------------------------------------------------------------
    explicit NatNetROS2Node()
    : Node("natnet_ros2_node")
    {
        // ----- Parameters --------------------------------------------------
        this->declare_parameter("server_ip",         "192.168.1.1");
        this->declare_parameter("client_ip",         "0.0.0.0");
        this->declare_parameter("command_port",      1510);
        this->declare_parameter("data_port",         1511);
        this->declare_parameter("connection_type",   std::string("unicast"));
        this->declare_parameter("multicast_address", std::string("239.255.42.99"));
        this->declare_parameter("frame_id",          "world");
        this->declare_parameter("debug",             false);

        // Parallel per-body arrays (flattened from natnet_config.yaml by the launch file).
        this->declare_parameter("body_names",   std::vector<std::string>{});
        this->declare_parameter("body_ids",     std::vector<int64_t>{});
        this->declare_parameter("body_topics",  std::vector<std::string>{});
        this->declare_parameter("body_pose",    std::vector<bool>{});
        this->declare_parameter("body_pose_cov", std::vector<bool>{});
        this->declare_parameter("body_position_covariance",    std::vector<double>{});
        this->declare_parameter("body_orientation_covariance", std::vector<double>{});

        // ----- Read parameters ---------------------------------------------
        const auto connect_cfg = natnet_ros2::make_connect_config(
            this->get_parameter("server_ip").as_string(),
            this->get_parameter("client_ip").as_string(),
            static_cast<uint16_t>(this->get_parameter("command_port").as_int()),
            static_cast<uint16_t>(this->get_parameter("data_port").as_int()),
            this->get_parameter("connection_type").as_string(),
            this->get_parameter("multicast_address").as_string());

        if (connect_cfg.connection_type !=
            this->get_parameter("connection_type").as_string())
        {
            RCLCPP_WARN(get_logger(),
                "Unknown connection_type '%s' — falling back to 'unicast'.",
                this->get_parameter("connection_type").as_string().c_str());
        }

        frame_id_ = this->get_parameter("frame_id").as_string();
        debug_    = this->get_parameter("debug").as_bool();

        const char * rn = std::getenv("ROBOT_NAME");
        robot_name_ = rn ? rn : "robot_1";

        build_body_configs();

        RCLCPP_INFO(get_logger(), "=========================================");
        RCLCPP_INFO(get_logger(), "NatNet ROS 2 Node");
        RCLCPP_INFO(get_logger(), "  robot_name:      %s", robot_name_.c_str());
        RCLCPP_INFO(get_logger(), "  server_ip:       %s", connect_cfg.server_ip.c_str());
        RCLCPP_INFO(get_logger(), "  command_port:    %d", static_cast<int>(connect_cfg.command_port));
        RCLCPP_INFO(get_logger(), "  connection_type: %s", connect_cfg.connection_type.c_str());
        if (natnet_ros2::is_multicast(connect_cfg)) {
            RCLCPP_INFO(get_logger(), "  multicast_addr:  %s", connect_cfg.multicast_address.c_str());
        }
        RCLCPP_INFO(get_logger(), "  tracked bodies:  %zu", bodies_.size());
        RCLCPP_INFO(get_logger(), "=========================================");

        // Production client — NatNetClientAdapter wraps the SDK
        client_ = std::make_unique<natnet_ros2::NatNetClientAdapter>();
        connect_cfg_ = connect_cfg;

        // Try to connect now; keep retrying.
        if (!connect_and_setup(connect_cfg_)) {
            connect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&NatNetROS2Node::retry_connect, this));
        }
    }

    // -----------------------------------------------------------------------
    ~NatNetROS2Node()
    {
        if (client_) { client_->disconnect(); }
    }

    // -----------------------------------------------------------------------
    // Called from the NatNetClientAdapter's frame trampoline.
    // publish() and Clock::now() are thread-safe; bodies_ is immutable after init.
    // -----------------------------------------------------------------------
    void on_frame(const natnet_ros2::FrameSample & frame)
    {
        if (debug_) {
            RCLCPP_DEBUG(get_logger(), "Frame %d: %zu rigid bodies, ts=%.4f s",
                frame.frame_num, frame.bodies.size(), static_cast<double>(frame.timestamp));
        }

        const rclcpp::Time stamp = this->get_clock()->now();

        for (const auto & rb : frame.bodies) {
            if (!natnet_ros2::is_tracking_valid(rb.params)) {
                if (debug_) {
                    RCLCPP_DEBUG(get_logger(), "  RB id=%d: tracking invalid, skipping", rb.id);
                }
                continue;
            }

            const auto it = bodies_.find(rb.id);
            if (it == bodies_.end()) { continue; }  // not configured for this robot

            const natnet_ros2::PoseData pose = natnet_ros2::rb_to_pose(rb);
            const BodyConfig & body = it->second;

            if (body.publish_pose && body.pose_pub) {
                PoseStamped msg;
                msg.header.frame_id    = frame_id_;
                msg.header.stamp       = stamp;
                msg.pose.position.x    = pose.x;
                msg.pose.position.y    = pose.y;
                msg.pose.position.z    = pose.z;
                msg.pose.orientation.x = pose.qx;
                msg.pose.orientation.y = pose.qy;
                msg.pose.orientation.z = pose.qz;
                msg.pose.orientation.w = pose.qw;
                body.pose_pub->publish(msg);
            }

            if (body.publish_pose_cov && body.pose_cov_pub) {
                PoseWithCovarianceStamped cov_msg;
                cov_msg.header.frame_id         = frame_id_;
                cov_msg.header.stamp            = stamp;
                cov_msg.pose.pose.position.x    = pose.x;
                cov_msg.pose.pose.position.y    = pose.y;
                cov_msg.pose.pose.position.z    = pose.z;
                cov_msg.pose.pose.orientation.x = pose.qx;
                cov_msg.pose.pose.orientation.y = pose.qy;
                cov_msg.pose.pose.orientation.z = pose.qz;
                cov_msg.pose.pose.orientation.w = pose.qw;
                cov_msg.pose.covariance         = body.covariance;
                body.pose_cov_pub->publish(cov_msg);
            }
        }
    }

private:
    // -----------------------------------------------------------------------
    // Returns true once the handshake succeeds.
    bool connect_and_setup(const natnet_ros2::ConnectConfig & cfg)
    {
        const natnet_ros2::NegotiationResult neg =
            natnet_ros2::negotiate(*client_, cfg);

        if (!neg.ok) {
            RCLCPP_WARN(get_logger(), "%s", neg.log_message.c_str());
            return false;
        }

        if (neg.server_info.host_present) {
            RCLCPP_INFO(get_logger(), "%s", neg.log_message.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "%s", neg.log_message.c_str());
        }

        client_->set_frame_callback(
            [this](const natnet_ros2::FrameSample & f) { on_frame(f); });
        RCLCPP_INFO(get_logger(), "Frame callback registered — receiving mocap data.");
        connected_ = true;
        return true;
    }

    // -----------------------------------------------------------------------
    // Timer-driven reconnect.
    void retry_connect()
    {
        if (connected_) {
            if (connect_timer_) { connect_timer_->cancel(); }
            return;
        }
        RCLCPP_INFO(get_logger(),
            "NatNet not connected — retrying handshake to %s ...",
            connect_cfg_.server_ip.c_str());
        if (connect_and_setup(connect_cfg_) && connect_timer_) {
            connect_timer_->cancel();
        }
    }

    // -----------------------------------------------------------------------
    // Build the per-body config map + publishers from the parallel param arrays.
    // Publishers are created up front (config-driven), so streaming begins as soon
    // as frames arrive — no dependency on Motive's data-description handshake.
    // -----------------------------------------------------------------------
    void build_body_configs()
    {
        const auto names    = this->get_parameter("body_names").as_string_array();
        const auto ids      = this->get_parameter("body_ids").as_integer_array();
        const auto topics   = this->get_parameter("body_topics").as_string_array();
        const auto pose     = this->get_parameter("body_pose").as_bool_array();
        const auto pose_cov = this->get_parameter("body_pose_cov").as_bool_array();
        const auto pos_cov  = this->get_parameter("body_position_covariance").as_double_array();
        const auto ori_cov  = this->get_parameter("body_orientation_covariance").as_double_array();

        const std::size_t n = std::min(names.size(), ids.size());
        if (names.size() != ids.size()) {
            RCLCPP_WARN(get_logger(),
                "body_names (%zu) and body_ids (%zu) length mismatch — using %zu.",
                names.size(), ids.size(), n);
        }

        for (std::size_t i = 0; i < n; ++i) {
            BodyConfig body;
            body.id              = static_cast<int32_t>(ids[i]);
            body.rigid_body_name = names[i];
            body.publish_pose     = (i < pose.size())     ? pose[i]     : true;
            body.publish_pose_cov = (i < pose_cov.size()) ? pose_cov[i] : true;

            const std::string relative = (i < topics.size()) ? topics[i] : std::string{};
            body.topic_base =
                natnet_ros2::body_topic_base(robot_name_, body.rigid_body_name, relative);

            body.covariance = natnet_ros2::build_covariance_6x6(
                cov_slice(pos_cov, i, _DEFAULT_POSITION_COVARIANCE),
                cov_slice(ori_cov, i, _DEFAULT_ORIENTATION_COVARIANCE));

            if (body.publish_pose) {
                body.pose_pub = this->create_publisher<PoseStamped>(body.topic_base, 10);
            }
            if (body.publish_pose_cov) {
                body.pose_cov_pub = this->create_publisher<PoseWithCovarianceStamped>(
                    body.topic_base + "/pose_cov", 10);
            }

            RCLCPP_INFO(get_logger(),
                "Tracking body id=%d name='%s' → %s (pose=%d pose_cov=%d)",
                static_cast<int>(body.id), body.rigid_body_name.c_str(),
                body.topic_base.c_str(),
                static_cast<int>(body.publish_pose),
                static_cast<int>(body.publish_pose_cov));

            bodies_.emplace(body.id, std::move(body));
        }
    }

    // -----------------------------------------------------------------------
    // Return the i-th 9-element covariance block from a flattened array, or the
    // built-in default when the slice is missing.
    static std::vector<double> cov_slice(
        const std::vector<double> & flat, std::size_t i, const std::vector<double> & fallback)
    {
        const std::size_t start = i * 9;
        if (flat.size() < start + 9) { return fallback; }
        return std::vector<double>(flat.begin() + start, flat.begin() + start + 9);
    }

    // -----------------------------------------------------------------------
    // Parameters / state
    std::string  frame_id_;
    bool         debug_     = false;
    std::string  robot_name_;

    std::unique_ptr<natnet_ros2::INatNetClient> client_;
    natnet_ros2::ConnectConfig connect_cfg_;
    bool connected_ = false;

    std::unordered_map<int32_t, BodyConfig> bodies_;

    rclcpp::TimerBase::SharedPtr connect_timer_;

    static const std::vector<double> _DEFAULT_POSITION_COVARIANCE;
    static const std::vector<double> _DEFAULT_ORIENTATION_COVARIANCE;
};

const std::vector<double> NatNetROS2Node::_DEFAULT_POSITION_COVARIANCE =
    {1.0e-6, 0., 0., 0., 1.0e-6, 0., 0., 0., 1.0e-6};
const std::vector<double> NatNetROS2Node::_DEFAULT_ORIENTATION_COVARIANCE =
    {3.0e-6, 0., 0., 0., 3.0e-6, 0., 0., 0., 3.0e-6};


// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NatNetROS2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

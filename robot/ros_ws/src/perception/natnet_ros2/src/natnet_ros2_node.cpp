// natnet_ros2_node.cpp
//
// ROS 2 NatNet SDK node for OptiTrack Motive integration.
//
// Published topics (per tracked rigid body):
//   /{robot_name}/perception/optitrack/{body_name}           → PoseStamped
//   /{robot_name}/perception/optitrack/{body_name}/pose_cov  → PoseWithCovarianceStamped
//
// Parameters (see config/natnet_config.yaml):
//   server_ip, client_ip, command_port, data_port,
//   body_name, body_id (-1 = all), publish_direct_optitrack,
//   frame_id, debug, position_covariance, orientation_covariance
//
// ROBOT_NAME is read from the environment variable set by AirStack's
// robot_name_map resolver at container startup.

#include <array>
#include <atomic>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

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

    struct BodyPublishers
    {
        rclcpp::Publisher<PoseStamped>::SharedPtr               pose_pub;
        rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;
    };

    // -----------------------------------------------------------------------
    explicit NatNetROS2Node()
    : Node("natnet_ros2_node")
    {
        // ----- Parameters --------------------------------------------------
        this->declare_parameter("server_ip",                "192.168.1.1");
        this->declare_parameter("client_ip",                "0.0.0.0");
        this->declare_parameter("command_port",             1510);
        this->declare_parameter("data_port",                1511);
        this->declare_parameter("connection_type",          std::string("unicast"));
        this->declare_parameter("multicast_address",        std::string("239.255.42.99"));
        this->declare_parameter("body_name",                "robot_1");
        this->declare_parameter("body_id",                  -1);
        this->declare_parameter("publish_direct_optitrack", true);
        this->declare_parameter("publish_to_mavros",        false);
        this->declare_parameter("frame_id",                 "world");
        this->declare_parameter("debug",                    false);
        this->declare_parameter(
            "position_covariance",
            std::vector<double>{0.1,0.,0., 0.,0.1,0., 0.,0.,0.1});
        this->declare_parameter(
            "orientation_covariance",
            std::vector<double>{0.01,0.,0., 0.,0.01,0., 0.,0.,0.01});

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

        body_name_      = this->get_parameter("body_name").as_string();
        body_id_        = static_cast<int32_t>(this->get_parameter("body_id").as_int());
        publish_direct_ = this->get_parameter("publish_direct_optitrack").as_bool();
        frame_id_       = this->get_parameter("frame_id").as_string();
        debug_          = this->get_parameter("debug").as_bool();

        covariance_6x6_ = natnet_ros2::build_covariance_6x6(
            this->get_parameter("position_covariance").as_double_array(),
            this->get_parameter("orientation_covariance").as_double_array());

        const char * rn = std::getenv("ROBOT_NAME");
        robot_name_ = rn ? rn : "robot_1";

        RCLCPP_INFO(get_logger(), "=========================================");
        RCLCPP_INFO(get_logger(), "NatNet ROS 2 Node");
        RCLCPP_INFO(get_logger(), "  robot_name:      %s", robot_name_.c_str());
        RCLCPP_INFO(get_logger(), "  server_ip:       %s", connect_cfg.server_ip.c_str());
        RCLCPP_INFO(get_logger(), "  command_port:    %d", static_cast<int>(connect_cfg.command_port));
        RCLCPP_INFO(get_logger(), "  connection_type: %s", connect_cfg.connection_type.c_str());
        if (natnet_ros2::is_multicast(connect_cfg)) {
            RCLCPP_INFO(get_logger(), "  multicast_addr:  %s", connect_cfg.multicast_address.c_str());
        }
        RCLCPP_INFO(get_logger(), "  body_id:         %d (%s)",
            static_cast<int>(body_id_),
            (body_id_ < 0) ? "track all" : "single body");
        RCLCPP_INFO(get_logger(), "=========================================");

        // Production client — NatNetClientAdapter wraps the SDK
        client_ = std::make_unique<natnet_ros2::NatNetClientAdapter>();
        connect_cfg_ = connect_cfg;

        // Try to connect now; if the server is not up yet keep retrying. The
        // NatNet server may legitimately start *after* the robot — e.g. the Isaac
        // Sim emulator only binds ~100 s into sim boot, and a real Motive PC may be
        // powered on after the drone. A one-shot connect would leave us dead for
        // the whole session, so retry on a timer until the first handshake lands.
        if (!connect_and_setup(connect_cfg_)) {
            connect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&NatNetROS2Node::retry_connect, this));
        }

        refresh_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NatNetROS2Node::refresh_descriptions_if_needed, this));
    }

    // -----------------------------------------------------------------------
    ~NatNetROS2Node()
    {
        if (client_) { client_->disconnect(); }
    }

    // -----------------------------------------------------------------------
    // Called from the NatNetClientAdapter's frame trampoline.
    // publish() and Clock::now() are thread-safe; pub_mutex_ guards map access.
    // -----------------------------------------------------------------------
    void on_frame(const natnet_ros2::FrameSample & frame)
    {
        if (natnet_ros2::model_list_changed(frame.params)) {
            needs_description_refresh_.store(true, std::memory_order_relaxed);
        }

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
            if (!natnet_ros2::should_publish_body(body_id_, rb.id)) { continue; }

            std::lock_guard<std::mutex> lock(pub_mutex_);

            const auto pub_it = publishers_.find(rb.id);
            if (pub_it == publishers_.end()) {
                needs_description_refresh_.store(true, std::memory_order_relaxed);
                continue;
            }

            const natnet_ros2::PoseData pose = natnet_ros2::rb_to_pose(rb);
            const BodyPublishers & bp = pub_it->second;

            if (publish_direct_ && bp.pose_pub) {
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
                bp.pose_pub->publish(msg);
            }

            if (bp.pose_cov_pub) {
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
                cov_msg.pose.covariance         = covariance_6x6_;
                bp.pose_cov_pub->publish(cov_msg);
            }
        }
    }

private:
    // -----------------------------------------------------------------------
    // Returns true once the handshake succeeds and the frame callback is live.
    // On failure it logs (WARN) and returns false so the caller can retry.
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

        refresh_descriptions_locked();

        client_->set_frame_callback(
            [this](const natnet_ros2::FrameSample & f) { on_frame(f); });
        RCLCPP_INFO(get_logger(), "Frame callback registered — receiving mocap data.");
        connected_ = true;
        return true;
    }

    // -----------------------------------------------------------------------
    // Timer-driven reconnect: fires every 2 s until the first handshake lands,
    // then cancels itself. Runs on the node's executor thread (same as the
    // refresh timer), so no extra locking versus single-threaded init.
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
    void refresh_descriptions_if_needed()
    {
        if (!needs_description_refresh_.exchange(false, std::memory_order_relaxed)) {
            return;
        }
        RCLCPP_INFO(get_logger(), "Model list change detected — refreshing data descriptions.");
        std::lock_guard<std::mutex> lock(pub_mutex_);
        refresh_descriptions_locked();
    }

    // -----------------------------------------------------------------------
    // Must be called with pub_mutex_ held (or from single-threaded init).
    // -----------------------------------------------------------------------
    void refresh_descriptions_locked()
    {
        if (!client_) { return; }

        // Always ensure the statically-configured body has a publisher
        if (body_id_ >= 0) {
            ensure_publisher_locked(body_id_, body_name_);
        }

        const auto bodies = client_->get_body_descriptors();
        int newly_created = 0;
        for (const auto & bd : bodies) {
            // Store name for every body (including skeleton bones)
            body_names_[bd.id] = bd.name;

            // Skip skeleton bones (parent_id >= 0)
            if (bd.parent_id >= 0) { continue; }

            // When tracking a single body, skip others
            if (!natnet_ros2::should_publish_body(body_id_, bd.id)) { continue; }

            if (ensure_publisher_locked(bd.id, bd.name)) { ++newly_created; }
        }

        if (newly_created > 0) {
            RCLCPP_INFO(get_logger(),
                "Data descriptions refreshed: %d new publisher(s) created.", newly_created);
        } else {
            RCLCPP_DEBUG(get_logger(), "Data descriptions refreshed: no new publishers.");
        }
    }

    // -----------------------------------------------------------------------
    bool ensure_publisher_locked(int32_t id, const std::string & name)
    {
        if (publishers_.count(id)) { return false; }

        const std::string topic_base =
            natnet_ros2::optitrack_topic_base(robot_name_, name);

        BodyPublishers bp;
        if (publish_direct_) {
            bp.pose_pub = this->create_publisher<PoseStamped>(topic_base, 10);
        }
        bp.pose_cov_pub = this->create_publisher<PoseWithCovarianceStamped>(
            natnet_ros2::optitrack_pose_cov_topic(robot_name_, name), 10);

        publishers_.emplace(id, std::move(bp));

        RCLCPP_INFO(get_logger(),
            "Publisher registered: id=%d  name='%s'  → %s[/pose_cov]",
            static_cast<int>(id), name.c_str(), topic_base.c_str());
        return true;
    }

    // -----------------------------------------------------------------------
    // Parameters / state
    std::string  body_name_;
    int32_t      body_id_          = -1;
    bool         publish_direct_   = true;
    std::string  frame_id_;
    bool         debug_            = false;
    std::string  robot_name_;

    std::array<double, 36> covariance_6x6_{};

    std::unique_ptr<natnet_ros2::INatNetClient> client_;
    natnet_ros2::ConnectConfig connect_cfg_;
    bool connected_ = false;

    std::mutex pub_mutex_;
    std::unordered_map<int32_t, std::string>    body_names_;
    std::unordered_map<int32_t, BodyPublishers> publishers_;

    std::atomic<bool> needs_description_refresh_{false};
    rclcpp::TimerBase::SharedPtr refresh_timer_;
    rclcpp::TimerBase::SharedPtr connect_timer_;
};


// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NatNetROS2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

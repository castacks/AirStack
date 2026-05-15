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
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

// NatNet SDK (bundled in include/natnet/, lib/libNatNet.so)
#include "NatNetClient.h"
#include "NatNetCAPI.h"
#include "NatNetTypes.h"

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


// Forward declaration
class NatNetROS2Node;

// ---------------------------------------------------------------------------
// SDK log callback — registered globally, routes to ROS logger when available.
// ---------------------------------------------------------------------------
static NatNetROS2Node* g_node_ptr = nullptr;

void NATNET_CALLCONV natnet_log_callback(Verbosity level, const char* message);
void NATNET_CALLCONV frame_callback(sFrameOfMocapData* data, void* pUserContext);


// ---------------------------------------------------------------------------
// NatNetROS2Node
// ---------------------------------------------------------------------------
class NatNetROS2Node : public rclcpp::Node
{
public:
    using PoseStamped              = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

    struct BodyPublishers
    {
        rclcpp::Publisher<PoseStamped>::SharedPtr              pose_pub;
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
        // "unicast" or "multicast"
        this->declare_parameter("connection_type",          std::string("unicast"));
        // Multicast group address used when connection_type = "multicast".
        // Must match Motive's "Data Streaming" multicast address setting.
        // OptiTrack default: 239.255.42.99  (NATNET_DEFAULT_MULTICAST_ADDRESS)
        this->declare_parameter("multicast_address",
            std::string(NATNET_DEFAULT_MULTICAST_ADDRESS));
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
        server_ip_         = this->get_parameter("server_ip").as_string();
        client_ip_         = this->get_parameter("client_ip").as_string();
        command_port_      = static_cast<uint16_t>(this->get_parameter("command_port").as_int());
        data_port_         = static_cast<uint16_t>(this->get_parameter("data_port").as_int());
        connection_type_   = this->get_parameter("connection_type").as_string();
        multicast_address_ = this->get_parameter("multicast_address").as_string();
        body_name_         = this->get_parameter("body_name").as_string();
        body_id_           = static_cast<int32_t>(this->get_parameter("body_id").as_int());
        publish_direct_    = this->get_parameter("publish_direct_optitrack").as_bool();
        frame_id_          = this->get_parameter("frame_id").as_string();
        debug_             = this->get_parameter("debug").as_bool();

        // Validate connection_type
        if (connection_type_ != "unicast" && connection_type_ != "multicast") {
            RCLCPP_WARN(get_logger(),
                "Unknown connection_type '%s' — falling back to 'unicast'.",
                connection_type_.c_str());
            connection_type_ = "unicast";
        }

        const auto pos_cov = this->get_parameter("position_covariance").as_double_array();
        const auto ori_cov = this->get_parameter("orientation_covariance").as_double_array();
        build_covariance_6x6(pos_cov, ori_cov);

        // ROBOT_NAME: set by AirStack's robot_name_map resolver at container startup.
        const char* rn = std::getenv("ROBOT_NAME");
        robot_name_ = rn ? rn : "robot_1";

        RCLCPP_INFO(get_logger(), "=========================================");
        RCLCPP_INFO(get_logger(), "NatNet ROS 2 Node");
        RCLCPP_INFO(get_logger(), "  robot_name:        %s", robot_name_.c_str());
        RCLCPP_INFO(get_logger(), "  server_ip:         %s", server_ip_.c_str());
        RCLCPP_INFO(get_logger(), "  client_ip:         %s", client_ip_.c_str());
        RCLCPP_INFO(get_logger(), "  command_port:      %d", static_cast<int>(command_port_));
        RCLCPP_INFO(get_logger(), "  data_port:         %d", static_cast<int>(data_port_));
        RCLCPP_INFO(get_logger(), "  connection_type:   %s", connection_type_.c_str());
        if (connection_type_ == "multicast") {
            RCLCPP_INFO(get_logger(), "  multicast_address: %s", multicast_address_.c_str());
        }
        RCLCPP_INFO(get_logger(), "  body_name:         %s", body_name_.c_str());
        RCLCPP_INFO(get_logger(), "  body_id:           %d (%s)",
            static_cast<int>(body_id_),
            (body_id_ < 0) ? "track all" : "single body");
        RCLCPP_INFO(get_logger(), "  frame_id:          %s", frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "=========================================");

        // Route SDK log messages to ROS logger (set before Connect)
        NatNet_SetLogCallback(natnet_log_callback);

        // Connect and pre-create publishers
        connect();

        // 1 Hz timer: refreshes data descriptions when Motive signals a model-list change.
        refresh_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NatNetROS2Node::refresh_descriptions_if_needed, this));
    }

    // -----------------------------------------------------------------------
    ~NatNetROS2Node()
    {
        g_node_ptr = nullptr;
        if (client_) {
            client_->Disconnect();
        }
    }

    // -----------------------------------------------------------------------
    // Called from frame_callback() on the NatNet SDK receive thread.
    // rclcpp::Publisher::publish() is thread-safe; Clock::now() is thread-safe.
    // pub_mutex_ serialises map lookups with the refresh timer that may call
    // create_publisher() on the spin thread.
    // -----------------------------------------------------------------------
    void on_frame(sFrameOfMocapData* data)
    {
        if (!data) return;

        // Bit 1 = model list changed; schedule a re-fetch on the spin thread.
        if (data->params & 0x02) {
            needs_description_refresh_.store(true, std::memory_order_relaxed);
        }

        if (debug_) {
            RCLCPP_DEBUG(get_logger(), "Frame %d: %d rigid bodies, ts=%.4f s",
                data->iFrame, data->nRigidBodies, data->fTimestamp);
        }

        const rclcpp::Time stamp = this->get_clock()->now();

        for (int i = 0; i < data->nRigidBodies; ++i) {
            const sRigidBodyData& rb = data->RigidBodies[i];

            // Bit 0 of params = tracking valid
            if (!(rb.params & 0x01)) {
                if (debug_) {
                    RCLCPP_DEBUG(get_logger(), "  RB id=%d: tracking invalid, skipping", rb.ID);
                }
                continue;
            }

            // Filter by configured body_id when not tracking all bodies
            if (body_id_ >= 0 && rb.ID != body_id_) continue;

            std::lock_guard<std::mutex> lock(pub_mutex_);

            const auto pub_it = publishers_.find(rb.ID);
            if (pub_it == publishers_.end()) {
                // Publisher not yet created (body appeared before first description fetch).
                // The refresh timer will create it within 1 s.
                needs_description_refresh_.store(true, std::memory_order_relaxed);
                continue;
            }

            const BodyPublishers& bp = pub_it->second;

            if (publish_direct_ && bp.pose_pub) {
                PoseStamped msg;
                msg.header.frame_id     = frame_id_;
                msg.header.stamp        = stamp;
                msg.pose.position.x     = static_cast<double>(rb.x);
                msg.pose.position.y     = static_cast<double>(rb.y);
                msg.pose.position.z     = static_cast<double>(rb.z);
                msg.pose.orientation.x  = static_cast<double>(rb.qx);
                msg.pose.orientation.y  = static_cast<double>(rb.qy);
                msg.pose.orientation.z  = static_cast<double>(rb.qz);
                msg.pose.orientation.w  = static_cast<double>(rb.qw);
                bp.pose_pub->publish(msg);
            }

            if (bp.pose_cov_pub) {
                PoseWithCovarianceStamped cov_msg;
                cov_msg.header.frame_id             = frame_id_;
                cov_msg.header.stamp                = stamp;
                cov_msg.pose.pose.position.x        = static_cast<double>(rb.x);
                cov_msg.pose.pose.position.y        = static_cast<double>(rb.y);
                cov_msg.pose.pose.position.z        = static_cast<double>(rb.z);
                cov_msg.pose.pose.orientation.x     = static_cast<double>(rb.qx);
                cov_msg.pose.pose.orientation.y     = static_cast<double>(rb.qy);
                cov_msg.pose.pose.orientation.z     = static_cast<double>(rb.qz);
                cov_msg.pose.pose.orientation.w     = static_cast<double>(rb.qw);
                cov_msg.pose.covariance             = covariance_6x6_;
                bp.pose_cov_pub->publish(cov_msg);
            }
        }
    }

private:
    // -----------------------------------------------------------------------
    void connect()
    {
        client_ = std::make_unique<NatNetClient>();

        sNatNetClientConnectParams params;
        params.serverAddress     = server_ip_.c_str();
        params.localAddress      = client_ip_.c_str();
        params.serverCommandPort = command_port_;
        params.serverDataPort    = data_port_;

        if (connection_type_ == "multicast") {
            params.connectionType   = ConnectionType_Multicast;
            params.multicastAddress = multicast_address_.c_str();
        } else {
            params.connectionType   = ConnectionType_Unicast;
            params.multicastAddress = nullptr;
        }

        const ErrorCode err = client_->Connect(params);
        if (err != ErrorCode_OK) {
            RCLCPP_ERROR(get_logger(),
                "NatNetClient::Connect failed (ErrorCode %d). "
                "Check server_ip, ports, and firewall on the Motive PC.",
                static_cast<int>(err));
            return;
        }

        // Log server details
        sServerDescription desc;
        memset(&desc, 0, sizeof(desc));
        if (client_->GetServerDescription(&desc) == ErrorCode_OK && desc.HostPresent) {
            RCLCPP_INFO(get_logger(),
                "Connected to Motive '%s' v%d.%d (NatNet %d.%d) at %s",
                desc.szHostApp,
                static_cast<int>(desc.HostAppVersion[0]),
                static_cast<int>(desc.HostAppVersion[1]),
                static_cast<int>(desc.NatNetVersion[0]),
                static_cast<int>(desc.NatNetVersion[1]),
                server_ip_.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "Connected to %s but GetServerDescription returned no host info.",
                server_ip_.c_str());
        }

        // Pre-create publishers for all known rigid bodies
        fetch_descriptions_and_create_publishers();

        // Register frame callback last — frames start arriving immediately
        client_->SetFrameReceivedCallback(frame_callback, this);
        RCLCPP_INFO(get_logger(), "Frame callback registered — receiving mocap data.");
    }

    // -----------------------------------------------------------------------
    // Fetches sDataDescriptions from Motive and creates publishers for every
    // rigid body asset.  Called on the spin thread (constructor + timer).
    // -----------------------------------------------------------------------
    void fetch_descriptions_and_create_publishers()
    {
        if (!client_) return;

        std::lock_guard<std::mutex> lock(pub_mutex_);

        // Always ensure the statically-configured body has a publisher, even if
        // GetDataDescriptionList hasn't returned yet or body_id is in a skeleton.
        if (body_id_ >= 0) {
            ensure_publishers_locked(body_id_, body_name_);
        }

        sDataDescriptions* desc_list = nullptr;
        const ErrorCode err = client_->GetDataDescriptionList(&desc_list);
        if (err != ErrorCode_OK || !desc_list) {
            RCLCPP_WARN(get_logger(),
                "GetDataDescriptionList failed (ErrorCode %d). "
                "Will retry when model list changes.", static_cast<int>(err));
            return;
        }

        int newly_created = 0;
        for (int i = 0; i < desc_list->nDataDescriptions; ++i) {
            const sDataDescription& dd = desc_list->arrDataDescriptions[i];
            if (dd.type != Descriptor_RigidBody || !dd.Data.RigidBodyDescription) continue;

            const sRigidBodyDescription& rb_desc = *dd.Data.RigidBodyDescription;

            // Store name for all bodies (including skeleton bones) for lookup in on_frame
            body_names_[rb_desc.ID] = rb_desc.szName;

            // Skip skeleton bones (they have a valid parentID)
            if (rb_desc.parentID >= 0) continue;

            // When tracking a single body, skip others to avoid noisy topics
            if (body_id_ >= 0 && rb_desc.ID != body_id_) continue;

            if (ensure_publishers_locked(rb_desc.ID, rb_desc.szName)) {
                ++newly_created;
            }
        }

        NatNet_FreeDescriptions(desc_list);

        if (newly_created > 0) {
            RCLCPP_INFO(get_logger(),
                "Data descriptions refreshed: %d new publisher(s) created.", newly_created);
        } else {
            RCLCPP_DEBUG(get_logger(), "Data descriptions refreshed: no new publishers.");
        }
    }

    // -----------------------------------------------------------------------
    // Creates PoseStamped + PoseWithCovarianceStamped publishers for a body
    // if they don't exist yet.  Returns true if publishers were created.
    // Must be called with pub_mutex_ held.
    // -----------------------------------------------------------------------
    bool ensure_publishers_locked(int32_t id, const std::string& name)
    {
        if (publishers_.count(id)) return false;

        const std::string topic_base =
            "/" + robot_name_ + "/perception/optitrack/" + name;

        BodyPublishers bp;
        if (publish_direct_) {
            bp.pose_pub = this->create_publisher<PoseStamped>(topic_base, 10);
        }
        bp.pose_cov_pub = this->create_publisher<PoseWithCovarianceStamped>(
            topic_base + "/pose_cov", 10);

        publishers_.emplace(id, std::move(bp));

        RCLCPP_INFO(get_logger(),
            "Publisher registered: id=%d  name='%s'  → %s[/pose_cov]",
            static_cast<int>(id), name.c_str(), topic_base.c_str());
        return true;
    }

    // -----------------------------------------------------------------------
    // Called by the 1 Hz timer on the spin thread.
    // -----------------------------------------------------------------------
    void refresh_descriptions_if_needed()
    {
        if (!needs_description_refresh_.exchange(false, std::memory_order_relaxed)) {
            return;
        }
        RCLCPP_INFO(get_logger(), "Model list change detected — refreshing data descriptions.");
        fetch_descriptions_and_create_publishers();
    }

    // -----------------------------------------------------------------------
    void build_covariance_6x6(
        const std::vector<double>& pos_cov,
        const std::vector<double>& ori_cov)
    {
        covariance_6x6_.fill(0.0);
        const int np = static_cast<int>(pos_cov.size());
        const int no = static_cast<int>(ori_cov.size());
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (r * 3 + c < np)
                    covariance_6x6_[r * 6 + c] = pos_cov[r * 3 + c];
                if (r * 3 + c < no)
                    covariance_6x6_[(r + 3) * 6 + (c + 3)] = ori_cov[r * 3 + c];
            }
        }
    }

    // -----------------------------------------------------------------------
    // Parameters
    std::string  server_ip_;
    std::string  client_ip_;
    uint16_t     command_port_     = NATNET_DEFAULT_PORT_COMMAND;
    uint16_t     data_port_        = NATNET_DEFAULT_PORT_DATA;
    std::string  connection_type_  = "unicast";
    std::string  multicast_address_ = NATNET_DEFAULT_MULTICAST_ADDRESS;
    std::string  body_name_;
    int32_t      body_id_          = -1;
    bool         publish_direct_   = true;
    std::string  frame_id_;
    bool         debug_            = false;
    std::string  robot_name_;

    std::array<double, 36> covariance_6x6_{};

    // NatNet SDK client
    std::unique_ptr<NatNetClient> client_;

    // Publisher management — protected by pub_mutex_.
    // Both on_frame() (SDK thread) and fetch_descriptions_and_create_publishers()
    // (spin thread) must hold this mutex before accessing these maps.
    std::mutex pub_mutex_;
    std::unordered_map<int32_t, std::string>    body_names_;
    std::unordered_map<int32_t, BodyPublishers> publishers_;

    // Set by on_frame when Motive signals model list changed (params bit 1).
    // Consumed and cleared by the 1 Hz refresh timer on the spin thread.
    std::atomic<bool> needs_description_refresh_{false};

    rclcpp::TimerBase::SharedPtr refresh_timer_;
};


// ---------------------------------------------------------------------------
// SDK log callback — filters verbose Info/Debug messages unless needed.
// ---------------------------------------------------------------------------
void NATNET_CALLCONV natnet_log_callback(Verbosity level, const char* message)
{
    if (!g_node_ptr) {
        if (level >= Verbosity_Warning) {
            fprintf(stderr, "[NatNet][%s] %s\n",
                (level == Verbosity_Warning) ? "WARN" : "ERROR", message);
        }
        return;
    }
    switch (level) {
        case Verbosity_Debug:
            RCLCPP_DEBUG(g_node_ptr->get_logger(), "[NatNet] %s", message); break;
        case Verbosity_Info:
            RCLCPP_DEBUG(g_node_ptr->get_logger(), "[NatNet] %s", message); break;
        case Verbosity_Warning:
            RCLCPP_WARN(g_node_ptr->get_logger(),  "[NatNet] %s", message); break;
        case Verbosity_Error:
            RCLCPP_ERROR(g_node_ptr->get_logger(), "[NatNet] %s", message); break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// Frame callback — called on the NatNet SDK receive thread.
// ---------------------------------------------------------------------------
void NATNET_CALLCONV frame_callback(sFrameOfMocapData* data, void* pUserContext)
{
    if (pUserContext) {
        static_cast<NatNetROS2Node*>(pUserContext)->on_frame(data);
    }
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NatNetROS2Node>();
    // Set global after construction so the log callback has a valid pointer.
    g_node_ptr = node.get();

    rclcpp::spin(node);

    g_node_ptr = nullptr;
    rclcpp::shutdown();
    return 0;
}

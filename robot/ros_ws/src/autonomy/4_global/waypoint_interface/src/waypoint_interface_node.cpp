#include <cmath>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class WaypointInterfaceNode : public rclcpp::Node {
   public:
    WaypointInterfaceNode() : Node("waypoint_interface") {
        this->declare_parameter<double>("lookahead_time", 5.0);
        lookahead_time_ = this->get_parameter("lookahead_time").as_double();

        subscription_ref_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan_reference", 10,
            std::bind(&WaypointInterfaceNode::global_plan_reference_callback, this, _1));
        subscription_eta_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan_eta", 10,
            std::bind(&WaypointInterfaceNode::global_plan_eta_callback, this, _1));

        // Publisher for the current waypoint index
        waypoint_index_publisher_ =
            this->create_publisher<std_msgs::msg::Int32>("current_waypoint_index", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&WaypointInterfaceNode::update_position, this));
    }

   private:
    void global_plan_reference_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        global_plan_reference_ = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Received global plan reference with sparse waypoints.");
    }

    void global_plan_eta_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        global_plan_eta_ = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Received global plan eta with dense waypoints.");
    }

    void update_position() {
        this->get_parameter("lookahead_time", lookahead_time_);
        RCLCPP_INFO(this->get_logger(), "Checking waypoints within lookahead time = %f",
                    lookahead_time_);
        track_waypoints(lookahead_time_);
    }

    void track_waypoints(double lookahead_time) {
        if (global_plan_reference_.empty() || global_plan_eta_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Global plans not yet received.");
            return;
        }

        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Time target_time = current_time + rclcpp::Duration::from_seconds(lookahead_time);

        int last_close_waypoint_idx = -1;

        for (const auto &pose_eta : global_plan_eta_) {
            if (rclcpp::Time(pose_eta.header.stamp) > target_time) {
                break;
            }

            for (size_t i = 0; i < global_plan_reference_.size(); ++i) {
                if (is_close(pose_eta.pose.position, global_plan_reference_[i].pose.position)) {
                    last_close_waypoint_idx = i;
                }
            }
        }

        std_msgs::msg::Int32 waypoint_index_msg;

        if (last_close_waypoint_idx == -1 ||
            last_close_waypoint_idx >= static_cast<int>(global_plan_reference_.size()) - 1) {
            RCLCPP_INFO(this->get_logger(),
                        "Robot is not within close range of any reference waypoints.");
            waypoint_index_msg.data = -1;  // No waypoint in range
        } else {
            RCLCPP_INFO(this->get_logger(), "Robot is traveling between waypoints %d and %d.",
                        last_close_waypoint_idx, last_close_waypoint_idx + 1);
            waypoint_index_msg.data = last_close_waypoint_idx;
        }

        // Publish the current waypoint index
        waypoint_index_publisher_->publish(waypoint_index_msg);
    }

    bool is_close(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const {
        double dist = distance(p1, p2);
        return dist < close_range_threshold_;
    }

    double distance(const geometry_msgs::msg::Point &p1,
                    const geometry_msgs::msg::Point &p2) const {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) +
                         std::pow(p1.z - p2.z, 2));
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_ref_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_eta_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_index_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> global_plan_reference_;
    std::vector<geometry_msgs::msg::PoseStamped> global_plan_eta_;

    double lookahead_time_;
    const double close_range_threshold_ = 0.5;  // Distance threshold for "close range" in meters
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

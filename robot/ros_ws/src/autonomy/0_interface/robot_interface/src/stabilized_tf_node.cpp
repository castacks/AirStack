#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StabilizedTFNode : public rclcpp::Node
{
public:
    StabilizedTFNode() : Node("stabilized_tf_node")
    {
        // Initialize the TF listener and broadcaster
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Declare parameters for the parent and child frames
        parent_frame_ = declare_parameter<std::string>("parent_frame", "map");
        child_frame_ = declare_parameter<std::string>("child_frame", "base_link");

        // Timer to update the stabilized frame at a fixed rate
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&StabilizedTFNode::publishStabilizedFrame, this));
    }

private:
    void publishStabilizedFrame()
    {
        // Lookup the original transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", parent_frame_.c_str(), child_frame_.c_str(), ex.what());
            return;
        }

        // Extract and zero out roll and pitch from the orientation
        tf2::Quaternion q_orig, q_stabilized;
        tf2::fromMsg(transform_stamped.transform.rotation, q_orig);

        // Get roll, pitch, and yaw from the original orientation
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

        // Set roll and pitch to zero, keeping only yaw
        q_stabilized.setRPY(0.0, 0.0, yaw);

        // Create a stabilized transform with the modified orientation
        geometry_msgs::msg::TransformStamped stabilized_transform = transform_stamped;
        stabilized_transform.child_frame_id = child_frame_ + "_stabilized"; // Naming convention
        stabilized_transform.transform.rotation = tf2::toMsg(q_stabilized);

        // Broadcast the stabilized transform
        tf_broadcaster_->sendTransform(stabilized_transform);
    }

    std::string parent_frame_;
    std::string child_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StabilizedTFNode>());
    rclcpp::shutdown();
    return 0;
}

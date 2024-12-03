#include <airstack_common/ros2_helper.hpp>
#include <airstack_common/tflib.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief
 * Does several things
 * - if there's an odometry, republishes it with a new frame_id
 * - if there's an odometry, republishes it as a transform
 * - converts MAVROS odometry BEST_EFFORT to RELIABLE
 *
 */

class OdometryConversion : public rclcpp::Node {
   private:
    enum OdometryOutputType { NONE, TRANSFORM, OVERWRITE };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    tf2_ros::Buffer* tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::TransformBroadcaster* tf_broadcaster;

    bool odom_input_qos_is_best_effort;
    std::string new_frame_id;
    std::string new_child_frame_id;
    OdometryOutputType odometry_output_type;
    bool convert_odometry_to_transform;
    bool convert_odometry_to_stabilized_transform;
    bool restamp_now_pre, restamp_now_post;
    tf2::Quaternion odom_orientation_rotation_pre, odom_orientation_rotation_post;
    tf2::Vector3 odom_position_translation_pre, odom_position_translation_post;

   public:
    OdometryConversion() : Node("odometry_conversion") {
        RCLCPP_INFO_STREAM(get_logger(), "OdometryConversion node started");
        odom_input_qos_is_best_effort =
            airstack::get_param(this, "odom_input_qos_is_best_effort", false);
        new_frame_id = airstack::get_param(this, "new_frame_id", std::string(""));
        new_child_frame_id = airstack::get_param(this, "new_child_frame_id", std::string(""));
        odometry_output_type =
            (OdometryOutputType)airstack::get_param(this, "odometry_output_type", (int)NONE);
        convert_odometry_to_transform =
            airstack::get_param(this, "convert_odometry_to_transform", false);
        convert_odometry_to_stabilized_transform =
            airstack::get_param(this, "convert_odometry_to_stabilized_transform", false);
        restamp_now_pre = airstack::get_param(this, "restamp_now_pre", false);
        restamp_now_post = airstack::get_param(this, "restamp_now_post", false);
        odom_orientation_rotation_pre =
            tf2::Quaternion(airstack::get_param(this, "odometry_orientation_rotation_pre_x", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_pre_y", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_pre_z", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_pre_w", 1.));
        odom_position_translation_pre =
            tf2::Vector3(airstack::get_param(this, "odometry_position_translation_pre_x", 0.),
                         airstack::get_param(this, "odometry_position_translation_pre_y", 0.),
                         airstack::get_param(this, "odometry_position_translation_pre_z", 0.));
        odom_orientation_rotation_post =
            tf2::Quaternion(airstack::get_param(this, "odometry_orientation_rotation_post_x", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_post_y", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_post_z", 0.),
                            airstack::get_param(this, "odometry_orientation_rotation_post_w", 1.));
        odom_position_translation_post =
            tf2::Vector3(airstack::get_param(this, "odometry_position_translation_post_x", 0.),
                         airstack::get_param(this, "odometry_position_translation_post_y", 0.),
                         airstack::get_param(this, "odometry_position_translation_post_z", 0.));

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos.depth = 1;
        if (odom_input_qos_is_best_effort) qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry_in", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&OdometryConversion::odom_callback, this, std::placeholders::_1));
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry_out", 1);
        tf_buffer = new tf2_ros::Buffer(this->get_clock());
        tf_listener = new tf2_ros::TransformListener(*tf_buffer);
        tf_broadcaster = new tf2_ros::TransformBroadcaster(*this);

        RCLCPP_WARN_STREAM(get_logger(), "Use odometry estimate as TF: " << std::boolalpha << convert_odometry_to_transform);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        nav_msgs::msg::Odometry out_odom = *msg;

        if (restamp_now_pre) out_odom.header.stamp = get_clock()->now();

        out_odom.pose.pose.orientation = tflib::from_tf(
            tflib::to_tf(out_odom.pose.pose.orientation) * odom_orientation_rotation_pre);

        if (odometry_output_type == NONE) {
            // do nothing
        }
        // transform mode transforms the odometry to a new frame, expects the new frame to exist in
        // the same tf tree
        else if (odometry_output_type == TRANSFORM) {
            try {
                out_odom = tflib::transform_odometry(tf_buffer, out_odom, new_frame_id,
                                                     new_child_frame_id);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR_STREAM(
                    get_logger(), "TransformException while transforming odometry: " << ex.what());
                return;
            }
        }
        // overwrite mode simply overwrites the header with the desired frame id and child frame id
        else if (odometry_output_type == OVERWRITE) {
            out_odom.header.frame_id = new_frame_id;
            out_odom.child_frame_id = new_child_frame_id;
        } else {
            RCLCPP_ERROR_STREAM(get_logger(),
                                "Unsupported odometry output type: " << odometry_output_type);
        }

        if (restamp_now_post) out_odom.header.stamp = get_clock()->now();

        out_odom.pose.pose.orientation = tflib::from_tf(
            tflib::to_tf(out_odom.pose.pose.orientation) * odom_orientation_rotation_post);

        if (convert_odometry_to_transform) {
            geometry_msgs::msg::TransformStamped t = tf2::toMsg(tflib::to_tf(out_odom));
            t.child_frame_id = out_odom.child_frame_id;
            tf_broadcaster->sendTransform(t);
        }

        if (convert_odometry_to_stabilized_transform) {
            geometry_msgs::msg::TransformStamped t = tf2::toMsg(tflib::to_tf(out_odom));
            t.child_frame_id = out_odom.child_frame_id + "_stabilized";
            t.transform.rotation =
                tflib::from_tf(tflib::get_stabilized(tflib::to_tf(out_odom.pose.pose.orientation)));
            tf_broadcaster->sendTransform(t);
        }

        odom_pub->publish(out_odom);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryConversion>());
    rclcpp::shutdown();
    return 0;
}

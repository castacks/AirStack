#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudFrameFixer : public rclcpp::Node
{
public:
    CloudFrameFixer() : Node("cloud_frame_fixer")
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/tof_pc");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/tof_pc_fixed");
        fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "tof");

        // SensorDataQoS: 适合高频传感器（best effort, small depth）
        auto qos = rclcpp::SensorDataQoS();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, qos,
                                                                        std::bind(&CloudFrameFixer::cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Fixing frame_id on %s -> %s, fixed_frame='%s'",
                    input_topic_.c_str(), output_topic_.c_str(), fixed_frame_.c_str());
    }

private:
    void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        auto out = sensor_msgs::msg::PointCloud2(*msg);

        std::string ff = fixed_frame_;
        out.header.frame_id = ff;
        pub_->publish(out);
    }

    std::string input_topic_, output_topic_, fixed_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudFrameFixer>());
    rclcpp::shutdown();
    return 0;
}

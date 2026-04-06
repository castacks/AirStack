#include "../include/takeoff_hack_node.hpp"

TakeoffHackNode::TakeoffHackNode() : rclcpp::Node("takeoff_hack_node")
{
    takeoff_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_1/takeoff_landing_planner/takeoff_state", 1);
    landing_state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_1/takeoff_landing_planner/landing_state", 1);

    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/robot_1/interface/mavros/cmd/takeoff", rmw_qos_profile_services_default, service_callback_group_);

    command_server_ = this->create_service<airstack_msgs::srv::TakeoffLandingCommand>("/robot_1/takeoff_landing_planner/set_takeoff_landing_command",
                                                                                      std::bind(&TakeoffHackNode::set_takeoff_landing_command, this,
                                                                                                std::placeholders::_1, std::placeholders::_2));
    ardupilot_takeoff_server_ = this->create_service<std_srvs::srv::Trigger>("/robot_1/takeoff_landing_planner/ardupilot_takeoff",
                                                                             std::bind(&TakeoffHackNode::ardupilot_takeoff, this,
                                                                                       std::placeholders::_1, std::placeholders::_2));
}

void TakeoffHackNode::set_takeoff_landing_command(const airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr request,
                                                  airstack_msgs::srv::TakeoffLandingCommand::Response::SharedPtr response)
{
    auto current_command = request->command;

    if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::NONE)
    {
    }
    else if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::TAKEOFF)
    {
        RCLCPP_INFO_STREAM(get_logger(), "hack takeofflanding 1");
    }
    else if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::LAND)
    {
        RCLCPP_INFO_STREAM(get_logger(), "hack takeofflanding 2");
    }
    RCLCPP_INFO_STREAM(get_logger(), "hack takeofflanding end");

    response->accepted = true;

    std_msgs::msg::String takeoff_state;
    takeoff_state.data = "COMPLETE";
    takeoff_state_pub_->publish(takeoff_state);
}

void TakeoffHackNode::ardupilot_takeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    mavros_msgs::srv::CommandTOL::Request::SharedPtr takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_request->altitude = 5.0;

    std::cout << "hack ardupilot takeoff 1" << std::endl;
    RCLCPP_WARN(get_logger(), "[DEBUG] hack: takeoff_client service = %s", takeoff_client_->get_service_name());
    auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);
    takeoff_result.wait();
    std::cout << "hack ardupilot takeoff 2" << std::endl;
    response->success = takeoff_result.get()->success;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<TakeoffHackNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
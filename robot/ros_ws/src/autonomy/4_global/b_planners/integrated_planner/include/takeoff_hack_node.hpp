#ifndef TAKEOFF_HACK_H
#define TAKEOFF_HACK_H

#include "rclcpp/rclcpp.hpp"

#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class TakeoffHackNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr takeoff_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr landing_state_pub_;

    // services
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::Service<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr command_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ardupilot_takeoff_server_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

    void set_takeoff_landing_command(const airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr request,
                                     airstack_msgs::srv::TakeoffLandingCommand::Response::SharedPtr response);
    void ardupilot_takeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

public:
    TakeoffHackNode();
};

#endif
#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/torque_thrust.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace robot_interface {
  class RobotInterface {
  protected:
    std::shared_ptr<rclcpp::Node> node;
    
  public:
    // init function
    virtual void initialize(std::shared_ptr<rclcpp::Node> node){
      this->node = node;
    }

    // TODO add low thrust mode
    
    // control callbacks
    // TODO maybe make parameters const references?
    virtual void attitude_thrust_callback(mav_msgs::msg::AttitudeThrust msg){}
    virtual void rate_thrust_callback(mav_msgs::msg::RateThrust msg){}
    virtual void roll_pitch_yawrate_thrust_callback(mav_msgs::msg::RollPitchYawrateThrust msg){}
    virtual void torque_thrust_callback(mav_msgs::msg::TorqueThrust msg){}
    virtual void velocity_callback(geometry_msgs::msg::TwistStamped msg){}
    virtual void pose_callback(geometry_msgs::msg::PoseStamped msg){}
    
    // command functions
    virtual bool request_control() = 0;
    virtual bool arm() = 0;
    virtual bool disarm() = 0;
    virtual bool is_armed() = 0;
    virtual bool has_control() = 0;
    
    virtual ~RobotInterface(){}
    
  protected:
    RobotInterface(){}
  };
}

#endif

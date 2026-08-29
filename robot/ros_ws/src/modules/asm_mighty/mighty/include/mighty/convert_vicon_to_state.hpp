#pragma once

#include <mighty/mighty.hpp>

#include "rclcpp/rclcpp.hpp"

#include "dynus_interfaces/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "std_msgs/msg/header.hpp"

// Define the synchronization policy
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped,
                                                        geometry_msgs::msg::TwistStamped>
    MySyncPolicy;
// typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PoseStamped,
// geometry_msgs::msg::TwistStamped> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

/** @brief ROS 2 node that converts synchronized PoseStamped + TwistStamped to dynus_interfaces/State.
 *
 *  Subscribes to Vicon pose and twist topics via approximate time synchronization
 *  and republishes the fused data as a State message for the MIGHTY planner.
 */
class PoseTwistToStateNode : public rclcpp::Node {
 public:
  /** @brief Construct the node, set up synchronized subscribers and publisher. */
  PoseTwistToStateNode();

 private:
  void callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& pose_msg,
                const std::shared_ptr<const geometry_msgs::msg::TwistStamped>& twist_msg);

  // Synchronized subscription
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_pose_;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> sub_twist_;

  // Time synchronizer
  std::shared_ptr<Sync> pose_twist_sync_;

  // Publisher
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr pub_state_;
};

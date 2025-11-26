#include <droan_gl/global_plan.hpp>

GlobalPlan::GlobalPlan(rclcpp::Node* node, tf2_ros::Buffer* tf_buffer)
  : node(node)
  , tf_buffer(tf_buffer){
  current_global_plan_id = -1;
  next_global_plan_id = -1;
  target_frame = airstack::get_param(node, "target_frame", std::string("map"));
}

bool GlobalPlan::update_global_plan(){
  if(next_global_plan_id == -1)
    return false;
  
  if(next_global_plan_id != current_global_plan_id){
    global_plan = Trajectory(node, path);
    global_plan = global_plan.to_frame(target_frame, path.header.stamp);
    current_global_plan_id = next_global_plan_id;
  }
  
  return true;
}

void GlobalPlan::set_global_plan(const nav_msgs::msg::Path::SharedPtr msg){
  path = *msg;
  next_global_plan_id = current_global_plan_id + 1;
}

void GlobalPlan::trim(const airstack_msgs::msg::Odometry& msg){
  if(!update_global_plan())
    return;
  
  tf2::Vector3 look_ahead_position = tflib::to_tf(msg.pose.position);
  bool success = tflib::to_frame(tf_buffer, look_ahead_position,
				 msg.header.frame_id, global_plan.get_frame_id(),
				 msg.header.stamp, &look_ahead_position);
  global_plan.trim(look_ahead_position);
}

std::tuple<float, float> GlobalPlan::get_distance(float x, float y, float z){
  if(!update_global_plan())
    return std::make_tuple(-1.f, -1.f);
  
  tf2::Vector3 p(x, y, z);
  auto [valid, wp, index, path_distance] = global_plan.get_closest_point(p);

  if(!valid)
    return std::make_tuple(-1.f, -1.f);

  return std::make_tuple(wp.position().distance(p), path_distance);
}

void GlobalPlan::publish_vis(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub){
  if(!update_global_plan())
    return;
  
  pub->publish(global_plan.get_markers(node->now(), "global_plan", 0, 0, 1));
}

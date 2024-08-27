#include <behavior_executive/behavior_executive.hpp>

BehaviorExecutive::BehaviorExecutive()
  : Node("behavior_executive"){
  
  // conditions
  takeoff_commanded_condition = new bt::Condition("Takeoff Commanded", this);
  armed_condition = new bt::Condition("Armed", this);
  offboard_mode_condition = new bt::Condition("Offboard Mode", this);
  stationary_condition = new bt::Condition("Stationary", this);
  land_commanded_condition = new bt::Condition("Land Commanded", this);
  pause_commanded_condition = new bt::Condition("Pause Commanded", this);
  rewind_commanded_condition = new bt::Condition("Rewind Commanded", this);
  fixed_trajectory_condition = new bt::Condition("Fixed Trajectory Commanded", this);
  explore_condition = new bt::Condition("Explore Commanded", this);
  offboard_commanded_condition = new bt::Condition("Offboard Commanded", this);
  arm_commanded_condition = new bt::Condition("Arm Commanded", this);
  disarm_commanded_condition = new bt::Condition("Disarm Commanded", this);
  conditions.push_back(takeoff_commanded_condition);
  conditions.push_back(armed_condition);
  conditions.push_back(offboard_mode_condition);
  conditions.push_back(stationary_condition);
  conditions.push_back(land_commanded_condition);
  conditions.push_back(pause_commanded_condition);
  conditions.push_back(rewind_commanded_condition);
  conditions.push_back(fixed_trajectory_condition);
  conditions.push_back(explore_condition);
  conditions.push_back(offboard_commanded_condition);
  conditions.push_back(arm_commanded_condition);
  conditions.push_back(disarm_commanded_condition);

  // actions
  arm_action = new bt::Action("Arm", this);
  takeoff_action = new bt::Action("Takeoff", this);
  land_action = new bt::Action("Land", this);
  pause_action = new bt::Action("Pause", this);
  rewind_action = new bt::Action("Rewind", this);
  follow_fixed_trajectory_action = new bt::Action("Follow Fixed Trajectory", this);
  explore_action = new bt::Action("Explore", this);
  request_control_action = new bt::Action("Request Control", this);
  disarm_action = new bt::Action("Disarm", this);
  actions.push_back(arm_action);
  actions.push_back(takeoff_action);
  actions.push_back(land_action);
  actions.push_back(pause_action);
  actions.push_back(rewind_action);
  actions.push_back(follow_fixed_trajectory_action);
  actions.push_back(explore_action);
  actions.push_back(request_control_action);
  actions.push_back(disarm_action);


  // subscribers
  behavior_tree_commands_sub =
    this->create_subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>("behavior_tree_commands", 1,
									     std::bind(&BehaviorExecutive::bt_commands_callback,
										       this, std::placeholders::_1));
  is_armed_sub = this->create_subscription<std_msgs::msg::Bool>("is_armed", 1, std::bind(&BehaviorExecutive::is_armed_callback,
											 this, std::placeholders::_1));
  has_control_sub = this->create_subscription<std_msgs::msg::Bool>("has_control", 1, std::bind(&BehaviorExecutive::has_control_callback,
											       this, std::placeholders::_1));

  // publishers

  // services
  service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  robot_command_client = this->create_client<airstack_msgs::srv::RobotCommand>("robot_command",
									       rmw_qos_profile_services_default,
									       service_callback_group);
  trajectory_mode_client = this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode",
										   rmw_qos_profile_services_default,
										   service_callback_group);
  takeoff_landing_command_client = this->create_client<airstack_msgs::srv::TakeoffLandingCommand>("set_takeoff_landing_command",
												  rmw_qos_profile_services_default,
												  service_callback_group);

  
  // timers
  timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1./20.),
			       std::bind(&BehaviorExecutive::timer_callback, this));
}

void BehaviorExecutive::timer_callback(){
  std::cout << "running" << std::endl;
  
  if(request_control_action->is_active()){
    if(request_control_action->active_has_changed()){
      airstack_msgs::srv::RobotCommand::Request::SharedPtr request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
      request->command = airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL;
      
      auto result = robot_command_client->async_send_request(request);
      std::cout << "waiting be rc" << std::endl;
      result.wait();
      std::cout << "done be rc" << std::endl;
      if(result.get()->success)
	request_control_action->set_success();
      else
	request_control_action->set_failure();
    }
  }

  if(arm_action->is_active()){
    if(arm_action->active_has_changed()){
      airstack_msgs::srv::RobotCommand::Request::SharedPtr request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
      request->command = airstack_msgs::srv::RobotCommand::Request::ARM;
      
      auto result = robot_command_client->async_send_request(request);
      std::cout << "waiting be arm" << std::endl;
      result.wait();
      std::cout << "done be arm" << std::endl;
      if(result.get()->success)
	arm_action->set_success();
      else
	arm_action->set_failure();
    }
  }

  
  if(disarm_action->is_active()){
    if(disarm_action->active_has_changed()){
      airstack_msgs::srv::RobotCommand::Request::SharedPtr request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
      request->command = airstack_msgs::srv::RobotCommand::Request::DISARM;
      
      auto result = robot_command_client->async_send_request(request);
      std::cout << "waiting be arm" << std::endl;
      result.wait();
      std::cout << "done be arm" << std::endl;
      if(result.get()->success)
	disarm_action->set_success();
      else
	disarm_action->set_failure();
    }
  }

  if(takeoff_action->is_active()){
    std::cout << "takeoff" << std::endl;
    takeoff_action->set_running();
    if(takeoff_action->active_has_changed()){
      // put trajectory controller in track mode
      airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
	std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
      mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
      auto mode_result = trajectory_mode_client->async_send_request(mode_request);
      std::cout << "mode 1" << std::endl;
      mode_result.wait();
      std::cout << "mode 2" << std::endl;

      if(mode_result.get()->success){
	// send a takeoff command to ardupilot
	// TODO clean up variable names
	airstack_msgs::srv::RobotCommand::Request::SharedPtr request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
	request->command = airstack_msgs::srv::RobotCommand::Request::TAKEOFF;
	
	auto result = robot_command_client->async_send_request(request);
	std::cout << "waiting robot command takeoff" << std::endl;
	result.wait();
	std::cout << "done robot command takeoff" << std::endl;
	
	// send the takeoff trajectory
	if(result.get()->success){
	  airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr takeoff_request =
	    std::make_shared<airstack_msgs::srv::TakeoffLandingCommand::Request>();
	  takeoff_request->command = airstack_msgs::srv::TakeoffLandingCommand::Request::TAKEOFF;
	  auto takeoff_result = takeoff_landing_command_client->async_send_request(takeoff_request);
	  std::cout << "takeoff 1" << std::endl;
	  takeoff_result.wait();
	  std::cout << "takeoff 2" << std::endl;
	  if(takeoff_result.get()->accepted)
	    takeoff_action->set_success();
	  else
	    takeoff_action->set_failure();
	}
	else
	  takeoff_action->set_failure();
      }
      else
	takeoff_action->set_failure();
    }
  }
  
  for(bt::Condition* condition : conditions)
    condition->publish();
  for(bt::Action* action : actions)
    action->publish();
}

// callbacks

void BehaviorExecutive::bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg){
  for(int i = 0; i < msg.commands.size(); i++){
    std::string condition_name = msg.commands[i].condition_name;
    int status = msg.commands[i].status;

    for(int j = 0; j < conditions.size(); j++){
      bt::Condition* condition = conditions[j];
      if(condition_name == condition->get_label()){
        if(status == behavior_tree_msgs::msg::Status::SUCCESS)
          condition->set(true);
        else if(status == behavior_tree_msgs::msg::Status::FAILURE)
          condition->set(false);
      }
    }
  }
}

void BehaviorExecutive::is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg){
  armed_condition->set(msg->data);
}

void BehaviorExecutive::has_control_callback(const std_msgs::msg::Bool::SharedPtr msg){
  offboard_mode_condition->set(msg->data);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<BehaviorExecutive>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

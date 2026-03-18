#include <airstack_common/ros2_helper.hpp>

namespace airstack {


  
  static DynamicParamInfo dynamic_param_info;

  DynamicParamInfo* get_dynamic_param_info(){
    return &dynamic_param_info;
  }

  rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if(get_dynamic_param_info()->temp_node != NULL){
      for (const auto & param : params) {
	RCLCPP_INFO_STREAM(get_dynamic_param_info()->temp_node->get_logger(), param.get_name());

	if(get_dynamic_param_info()->dynamic_params.count(param.get_name()) > 0){
	  ParamInfo pi = get_dynamic_param_info()->dynamic_params[param.get_name()];

	  if(pi.type == rclcpp::ParameterType::PARAMETER_INTEGER)
	    *((int*)pi.variable) = param.as_int();
	  else if(pi.type == rclcpp::ParameterType::PARAMETER_DOUBLE)
	    *((double*)pi.variable) = param.as_double();
	  else if(pi.type == rclcpp::ParameterType::PARAMETER_STRING)
	    *((std::string*)pi.variable) = param.as_string();
	  else if(pi.type == rclcpp::ParameterType::PARAMETER_BOOL)
	    *((bool*)pi.variable) = param.as_bool();
	}
      }
    }
    
    return result;
  }
  
}

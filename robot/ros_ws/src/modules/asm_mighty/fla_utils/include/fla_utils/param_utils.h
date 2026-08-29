// Copyright 2016 Massachusetts Intitute of Technology

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace fla_utils {

template <typename T>
inline bool SafeGetParam(rclcpp::Node & node,
                         std::string const& param_name,
                         T & param_value) {
  if (!node.get_parameter(param_name, param_value)) {
    RCLCPP_ERROR(node.get_logger(), "Failed to find parameter: %s", param_name.c_str());
    return false;
  }
  return true;
}


inline bool StringGetParam(rclcpp::Node & node,
                         std::string const& param_name, std::string& param_value) {
  int param_value_int;
  double param_value_double;
  bool param_value_bool;
  std::string param_value_string;
  
  if(node.get_parameter(param_name, param_value_int)) {
  	param_value = std::to_string(param_value_int);
	return true;
  } else if(node.get_parameter(param_name, param_value_double)) {
  	param_value = std::to_string(param_value_double);
	return true;
  } if(node.get_parameter(param_name, param_value_bool)) {
  	param_value = param_value_bool ? "true" : "false";
	return true;
  } else if(node.get_parameter(param_name, param_value_string)) {
  	param_value = param_value_string;
	return true;
  } else {
	RCLCPP_ERROR(node.get_logger(), "Failed to find parameter: %s", param_name.c_str());
	return false;
  }
}


}  // namespace fla_utils

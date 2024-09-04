#ifndef _AIRSTACK_COMMON_H_
#define _AIRSTACK_COMMON_H_

#include <rclcpp/rclcpp.hpp>


namespace airstack {


  // parameters

  // return parameter
  
  template <typename T>
  inline T get_param(rclcpp::Node* node, std::string name, T default_value, bool* set) = delete;
  
  template <>
  inline int get_param(rclcpp::Node* node, std::string name, int default_value, bool* set){
    try{
      node->declare_parameter(name, rclcpp::PARAMETER_INTEGER);
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException& e){}
    rclcpp::Parameter param;
    bool s = node->get_parameter_or(name, param, rclcpp::Parameter(name, default_value));
    if(set != NULL)
      *set = s;
    return param.as_int();
  }
  
  template <>
  inline double get_param(rclcpp::Node* node, std::string name, double default_value, bool* set){
    try{
      node->declare_parameter(name, rclcpp::PARAMETER_DOUBLE);
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException& e){}
    rclcpp::Parameter param;
    bool s = node->get_parameter_or(name, param, rclcpp::Parameter(name, default_value));
    if(set != NULL)
      *set = s;
    return param.as_double();
  }
  
  template <>
  inline std::string get_param(rclcpp::Node* node, std::string name, std::string default_value, bool* set){
    try{
      node->declare_parameter(name, rclcpp::PARAMETER_STRING);
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException& e){}
    rclcpp::Parameter param;
    bool s = node->get_parameter_or(name, param, rclcpp::Parameter(name, default_value));
    if(set != NULL)
      *set = s;
    return param.as_string();
  }

  template <>
  inline bool get_param(rclcpp::Node* node, std::string name, bool default_value, bool* set){
    try{
      node->declare_parameter(name, rclcpp::PARAMETER_BOOL);
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException& e){}
    rclcpp::Parameter param;
    bool s = node->get_parameter_or(name, param, rclcpp::Parameter(name, default_value));
    if(set != NULL)
      *set = s;
    return param.as_bool();
  }
  
  template <typename T>
  inline T get_param(std::shared_ptr<rclcpp::Node> node, std::string name, T default_value, bool* set){
    return get_param(node.get(), name, default_value, set);
  }

  
  template <typename T>
  inline T get_param(rclcpp::Node* node, std::string name, T default_value){
    bool set;
    return get_param(node, name, default_value, &set);
  }
  
  template <typename T>
  inline T get_param(std::shared_ptr<rclcpp::Node> node, std::string name, T default_value){
    bool set;
    return get_param(node.get(), name, default_value, &set);
  }

  // services

  template <typename T>
  inline typename T::Response::SharedPtr sync_send_request(typename rclcpp::Client<T>::SharedPtr client,
						    typename T::Request::SharedPtr request){
    auto future_result = client->async_send_request(request);
    future_result.wait();
    return future_result;
  }
  
}

#endif

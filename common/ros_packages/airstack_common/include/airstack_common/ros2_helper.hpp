#ifndef _AIRSTACK_COMMON_H_
#define _AIRSTACK_COMMON_H_

#include <rclcpp/rclcpp.hpp>


namespace airstack {


  // parameters

  // return parameter
  
  template <typename T>
  inline T get_param(rclcpp::Node* node, std::string name, T default_value, bool* set) = delete;
  /*  
  template <>
  inline int get_param(rclcpp::Node* node, std::string name, int default_value, bool* set);
  template <>
  inline double get_param(rclcpp::Node* node, std::string name, double default_value, bool* set);
  template <>
  inline std::string get_param(rclcpp::Node* node, std::string name, std::string default_value, bool* set);
  template <>
  inline bool get_param(rclcpp::Node* node, std::string name, bool default_value, bool* set);
  */
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


  struct ParamInfo{
    void* variable;
    rclcpp::ParameterType type;
  };

  struct DynamicParamInfo {
    std::unordered_map<std::string, ParamInfo> dynamic_params;
    rclcpp::Node* temp_node = NULL;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
  };

  DynamicParamInfo* get_dynamic_param_info();

  rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params);
  
  template <typename T>
  inline void dynamic_param(rclcpp::Node* node, std::string name, T default_value, T* variable){
    get_dynamic_param_info()->temp_node = node;
    //rclcpp::ParameterValue pv = node->declare_parameter(name, rclcpp::ParameterValue(default_value));
    //*variable = default_value;
    *variable = get_param(node, name, default_value);

    ParamInfo pi;
    pi.variable = (void*)variable;
    pi.type = node->get_parameter(name).get_type();//pv.get_type();
    get_dynamic_param_info()->dynamic_params[name] = pi;

    if(!get_dynamic_param_info()->param_callback_handle)
      get_dynamic_param_info()->param_callback_handle = node->add_on_set_parameters_callback(&on_param_change);
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

#!/usr/bin/python3

import rclpy
#from rclpy.parameter import Parameter as prmtr
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import numpy as np
from rclpy.node import Node
from natnet_ros2.srv import MarkerPoses

class HelperNode(Node):
  def __init__(self):
    super().__init__('helper_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True,
                        start_parameter_services=True)

    self.marker_pose_cli = self.create_client(MarkerPoses,'get_marker_position')
    while not self.marker_pose_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    self.req = MarkerPoses.Request()

  def request_markerposes(self):
    self.future = self.marker_pose_cli.call_async(self.req)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()

  def call_set_parameters(self, node_name:str, param_dict:dict) -> bool:
    #parameters=param_dict
    parameters=[]
    for key,value in param_dict.items():
      value_type = str(type(value)).lower()
      if 'None' in value_type:
        self.get_logger().error(f"{value_type} is not supported!")
      elif 'bool' in value_type:
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)))
      elif 'int' in value_type:
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)))
      elif 'float' in value_type:
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)))
      elif 'str' in value_type:
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)))
      elif 'list' in value_type:
        if all(isinstance(n, bool) for n in value):
          parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY, bool_array_value=value)))
        elif all(isinstance(n, int) for n in value):
          parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=value)))
        elif all(isinstance(n, float) for n in value):
          parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=value)))
        elif all(isinstance(n, str) for n in value):
          parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=value)))
      else:
        self.get_logger().error(f"{value_type} is not supported!")
        raise TypeError(f"{value_type} is not supported!")

    # create client
    client = self.create_client(SetParameters, f'{node_name}/set_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = SetParameters.Request()
    request.parameters = parameters
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    if all(n.successful for n in response.results[:]): return True
    
    return False
  
  def call_get_parameters(self, node_name:str, parameter_names:list)->dict:
    param_dict = {}
    # create client
    client = self.create_client(GetParameters, f'{node_name}/get_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    for i in range(len(response.values)):#request.values[:]:
      if response.values[i].type==1:
        param_dict[parameter_names[i]]=response.values[i].bool_value
      elif response.values[i].type==2:
        param_dict[parameter_names[i]]=response.values[i].integer_value
      elif response.values[i].type==3:
        param_dict[parameter_names[i]]=response.values[i].double_value
      elif response.values[i].type==4:
        param_dict[parameter_names[i]]=response.values[i].string_value
      elif response.values[i].type==5:
        param_dict[parameter_names[i]]=response.values[i].byte_array_value
      elif response.values[i].type==6:
        param_dict[parameter_names[i]]=response.values[i].bool_array_value
      elif response.values[i].type==7:
        param_dict[parameter_names[i]]=response.values[i].integer_array_value
      elif response.values[i].type==8:
        param_dict[parameter_names[i]]=response.values[i].double_array_value
      elif response.values[i].type==9:
        param_dict[parameter_names[i]]=response.values[i].string_array_value
      else:
        self.get_logger().error(f"Type {response.values[i].type} is not supported!")
        raise TypeError(f"Type {response.values[i].type} is not supported!")

    return param_dict
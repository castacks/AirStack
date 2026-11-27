#!/usr/bin/env python3
"""Run via stdin in robot container. Passive topics and GetParameters only."""
import datetime
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

rclpy.init()
node = rclpy.create_node('rrm_m00b_read_only_diagnostics', enable_rosout=False,
                        start_parameter_services=False)
start = time.monotonic()
while time.monotonic() - start < 2:
    rclpy.spin_once(node, timeout_sec=0.1)
graph = dict(node.get_topic_names_and_types())
selected = {name: types for name, types in graph.items()
            if any(x in name for x in ('actuation_', 'mavros/state', 'mavros/extended_state',
                                      'vdb', 'odometry')) or name == '/clock'}
constructors = {'std_msgs/msg/Bool': Bool, 'std_msgs/msg/String': String,
                'mavros_msgs/msg/State': State, 'mavros_msgs/msg/ExtendedState': ExtendedState,
                'rosgraph_msgs/msg/Clock': Clock, 'visualization_msgs/msg/MarkerArray': MarkerArray}
channels = {}
subscriptions = []
for topic, types in selected.items():
    if len(types) != 1 or types[0] not in constructors:
        continue
    channels[topic] = {'type': types[0], 'samples': []}

    def callback(message, name=topic):
        item = {'receipt_monotonic': time.monotonic()}
        if isinstance(message, (Bool, String)):
            item['data'] = message.data
        elif isinstance(message, State):
            item.update(connected=message.connected, armed=message.armed, mode=message.mode)
        elif isinstance(message, ExtendedState):
            item.update(landed_state=message.landed_state, vtol_state=message.vtol_state)
        elif isinstance(message, Clock):
            item['clock_ns'] = message.clock.sec * 1000000000 + message.clock.nanosec
        elif isinstance(message, MarkerArray):
            item.update(markers=len(message.markers), points=sum(len(m.points) for m in message.markers))
        channels[name]['samples'].append(item)

    subscriptions.append(node.create_subscription(constructors[types[0]], topic, callback,
                                                  qos_profile_sensor_data))
queries = []
for service, names in [('/robot_1/interface/mavros/setpoint_raw/get_parameters', ['thrust_scaling']),
                       ('/robot_1/interface/robot_interface/get_parameters',
                        ['actuation_startup_timeout_s', 'mavros_actuation_config', 'is_ardupilot',
                         'use_sim_time'])]:
    client = node.create_client(GetParameters, service)
    item = {'service': service, 'parameters': names}
    if client.wait_for_service(timeout_sec=2):
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if future.done() and future.result():
            item['values'] = [{'name': name, 'type': value.type, 'double': value.double_value,
                               'integer': value.integer_value, 'string': value.string_value,
                               'bool': value.bool_value}
                              for name, value in zip(names, future.result().values)]
        else:
            item['error'] = 'No bounded GetParameters response'
    else:
        item['error'] = 'GetParameters service unavailable'
    queries.append(item)
    node.destroy_client(client)
sample_start = time.monotonic()
while time.monotonic() - sample_start < 12:
    rclpy.spin_once(node, timeout_sec=0.1)
end = time.monotonic()
for channel in channels.values():
    samples = channel['samples']
    channel['count'] = len(samples)
    channel['last_receipt_age_s'] = end - samples[-1]['receipt_monotonic'] if samples else None
    channel['max_receipt_gap_s'] = max((b['receipt_monotonic'] - a['receipt_monotonic']
                                      for a, b in zip(samples, samples[1:])), default=None)
endpoints = {}
for topic in channels:
    endpoints[topic] = [{'node_name': x.node_name, 'node_namespace': x.node_namespace,
                          'reliability': str(x.qos_profile.reliability),
                          'durability': str(x.qos_profile.durability)}
                         for x in node.get_publishers_info_by_topic(topic)]
print(json.dumps({'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  'duration_wall_s': end - start, 'ros_domain_id': 1,
                  'selected_graph_topics': selected, 'queries': queries, 'channels': channels,
                  'publisher_endpoints': endpoints,
                  'limitations': 'Passive telemetry and read-only ROS GetParameters only. No SetParameters, mode, arming, action or motion request. Header source freshness and physical ground truth not established; monotonic times are this collector process.'}, indent=2))
node.destroy_node()
rclpy.shutdown()

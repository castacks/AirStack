"""Small ROS helper, executed inside the AirStack robot container."""
import argparse
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("parameters", help="JSON dictionary of bridge parameters")
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node("ws2_condition_control")
    client=node.create_client(SetParametersAtomically,"/vision_planner/bridge/set_parameters_atomically")
    if not client.wait_for_service(timeout_sec=15):
        raise RuntimeError("vision bridge parameter service unavailable")
    request=SetParametersAtomically.Request()
    request.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in json.loads(args.parameters).items()]
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    if not future.done() or future.result() is None:
        raise RuntimeError("bridge parameter update timed out")
    result=future.result().result
    print(json.dumps({"ok":result.successful,"reason":result.reason}))
    node.destroy_node();rclpy.shutdown()
    if not result.successful:
        raise RuntimeError(result.reason)


if __name__=="__main__":main()

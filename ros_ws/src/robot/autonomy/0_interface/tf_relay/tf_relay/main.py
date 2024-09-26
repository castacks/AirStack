import rclpy
from tf_relay.tf_relay import TFRelay
from tf_relay.tf_static_relay import TFStaticRelay
from rclpy.executors import SingleThreadedExecutor
import sys

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    num_robots = 4
    namespace_prefix = 'robot_'

    if len(sys.argv) > 1:
        namespace_prefix = str(sys.argv[1])
        num_robots = int(sys.argv[2])

    tf_relays = []
    tf_static_relays = []

    try:
        for i in range(num_robots):
            robot_id = i + 1
            tf_relay = TFRelay(namespace_prefix + str(robot_id))
            tf_static_relay = TFStaticRelay(namespace_prefix + str(robot_id))
            tf_relays.append(tf_relay)
            tf_static_relays.append(tf_static_relay)

        for tf_relay in tf_relays:
            executor.add_node(tf_relay)
        
        for tf_static_relay in tf_static_relays:
            executor.add_node(tf_static_relay)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        executor.shutdown()


if __name__ == '__main__':
    main()

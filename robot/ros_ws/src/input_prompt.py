import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class InputPublisher(Node):
    def __init__(self):
        super().__init__('input_publisher')
        self.publisher_ = self.create_publisher(String, '/input_prompt', 10)
        self.current_input = ""
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.start()

    def publish_loop(self):
        msg = String()
        while self.running:
            msg.data = self.current_input
            self.publisher_.publish(msg)
            time.sleep(0.2)  # 5Hz

    def set_input(self, new_input):
        self.current_input = new_input

    def clear_input(self):
        self.current_input = ""

    def stop(self):
        self.running = False
        self.publish_thread.join()

def main():
    rclpy.init()
    node = InputPublisher()

    try:
        while True:
            print("\n=== Input Publisher ===")
            print(f"Current input: '{node.current_input}'")
            print("1. Set new input")
            print("2. Clear input")
            print("3. Exit")
            choice = input("Choose option: ").strip()
            if choice == '1':
                new_input = input("Enter new input: ")
                node.set_input(new_input)
            elif choice == '2':
                node.clear_input()
                print("Input cleared.")
            elif choice == '3':
                break
            else:
                print("Invalid choice.")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


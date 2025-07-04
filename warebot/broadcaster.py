#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from warebot.srv import TaskStatus

class Broadcaster(Node):
    def __init__(self):
        super().__init__('broadcaster')
        self.client = self.create_client(TaskStatus, 'task_status')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.timer = self.create_timer(5.0, self.request_status)
        self.task_counter = 1

    def request_status(self):
        request = TaskStatus.Request()
        request.task_id = self.task_counter
        
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
        
        self.get_logger().info(f"Requested status for task {self.task_counter}")
        self.task_counter += 1

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received status: {response.status}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    broadcaster = Broadcaster()
    
    try:
        rclpy.spin(broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
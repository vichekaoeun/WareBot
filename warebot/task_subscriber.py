import rclpy
import json
from std_msgs.msg import String
from rclpy.node import Node
from queue import Queue

class TaskSubscriber(Node):
    def __init__(self, task_queue):
        super().__init__('task_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/delivery_task',
            self.listener_callback,
            10   
        )
        self.task_queue = task_queue
    
    def listener_callback(self, msg):
        try:
            task_data = json.loads(msg.data)
            self.task_queue.put(task_data)
            self.get_logger().info(f"Task added to queue: {task_data}")
        except Exception as e:
            self.get_logger().error(f"Bad JSON: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    task_queue = Queue()
    task_subscriber = TaskSubscriber(task_queue)
    rclpy.spin(task_subscriber)
    task_subscriber.destroy_node()
    rclpy.shutdown()
            
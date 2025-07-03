import rclpy
import json
from std_msgs.msg import String
from rclpy.node import Node
from queue import Queue, Empty
from rclpy.executors import MultiThreadedExecutor

class TaskSubscriber(Node):
    def __init__(self, task_queue):
        super().__init__('task_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/delivery_task',
            self.listener_callback,
            10
        )
        self.subscription
        self.task_queue = task_queue

    def listener_callback(self, msg):
        try:
            task_data = json.loads(msg.data)
            self.task_queue.put(task_data)
            self.get_logger().info(f"Task added to queue: {task_data}")
        except Exception as e:
            self.get_logger().error(f"Bad JSON: {e}")

class TaskExecutor(Node):
    def __init__(self, task_queue):
        super().__init__('task_executor')
        self.task_queue = task_queue
        self.create_timer(1.0, self.execute_task)

    def execute_task(self):
        try:
            task_data = self.task_queue.get_nowait()
            self.get_logger().info(f"Executing task: {task_data}")
            # TODO: Add robot navigation or fake delivery here
        except Empty:
            pass
    
def main(args=None):
    rclpy.init(args=args)
    task_queue = Queue()
    
    task_subscriber = TaskSubscriber(task_queue)
    task_executor = TaskExecutor(task_queue)

    executor = MultiThreadedExecutor()
    executor.add_node(task_subscriber)
    executor.add_node(task_executor)

    try:
        executor.spin()
    finally:
        task_subscriber.destroy_node()
        task_executor.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
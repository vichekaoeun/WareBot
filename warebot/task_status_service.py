#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from warebot.srv import TaskStatus

class TaskStatusService(Node):
    def __init__(self):
        super().__init__('task_status_service')
        self.srv = self.create_service(TaskStatus, 'task_status', self.handle_task_status)
        
    def handle_task_status(self, request, response):
        self.get_logger().info(f"Received task status request: {request.task_id}")
        
        # Here you would normally check the status of the task
        # For demonstration, we will just return a dummy status
        response.status = "Task completed successfully"
        
        self.get_logger().info(f"Returning status for task {request.task_id}: {response.status}")
        return response

def main(args=None):
    rclpy.init()
    task_status_service = TaskStatusService()
    rclpy.spin(task_status_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
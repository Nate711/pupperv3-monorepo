import rclpy
from rclpy.node import Node


class RosToolServer:
    def __init__(self):

        rclpy.init()
        self.node = Node("ros_tool_server")
        self.node.get_logger().info("ROS Tool Server has been started.")

    def queue_activate(self):
        self.node.get_logger().info("Activating motors")
        return "Motor activate queue up. Will be executed soon."

    def queue_deactivate(self):
        self.node.get_logger().info("Deactivating motors")
        return "Motor deactivate queue up. Will be executed soon."

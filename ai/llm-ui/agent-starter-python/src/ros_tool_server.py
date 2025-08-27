from typing import Tuple
import rclpy
from rclpy.node import Node
import logging
from controller_manager_msgs.srv import SwitchController

logger = logging.getLogger("ros_tool_server")
AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}


class RosToolServer:
    def __init__(self):

        rclpy.init()
        self.node = Node("ros_tool_server")
        logger.info("ROS Tool Server has been started.")

        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.current_gait = "4-legged"  # default gait

    async def queue_activate(self):
        logger.info("Activating motors")
        return await self.activate()

    async def queue_deactivate(self):
        logger.info("Deactivating motors")
        return await self.deactivate()

    async def deactivate(self) -> Tuple[bool, str]:
        req = SwitchController.Request()
        req.activate_controllers = []
        req.deactivate_controllers = list(AVAILABLE_CONTROLLERS)
        req.strictness = 1

        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done() and future.result().ok:
            logger.info("🤖 Robot DEACTIVATED")
            return True, "Robot deactivated successfully"
        else:
            logger.error("❌ Failed to deactivate robot")
            return False, "Failed to deactivate robot - controller switch failed"

    async def activate(self) -> Tuple[bool, str]:

        # Use the same logic as openai_bridge activate function
        req = SwitchController.Request()
        controller_name = CONTROLLER_NAME_MAP[self.current_gait]

        req.activate_controllers = [controller_name]
        req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
        req.strictness = 1

        # Call the service synchronously in the async context
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done() and future.result().ok:
            logger.info("🤖 Robot ACTIVATED")
            return True, f"Robot activated successfully with {self.current_gait} gait"
        else:
            logger.error("❌ Failed to activate robot")
            return False, "Failed to activate robot - controller switch failed"

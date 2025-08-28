from typing import Tuple
import rclpy
from rclpy.node import Node
import logging
from controller_manager_msgs.srv import SwitchController
from dataclasses import dataclass
from geometry_msgs.msg import Twist

logger = logging.getLogger("ros_tool_server")
AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}


@dataclass
class MoveCfg:
    vx_threshold: float = 0.3
    vy_threshold: float = 0.3
    wz_threshold: float = 1.0
    vx_max: float = 0.75
    vy_max: float = 0.5
    wz_max: float = 2.0


@dataclass
class DefaultCfg:
    move_cfg: MoveCfg = MoveCfg()


DEFAULT_CFG = DefaultCfg()


class RosToolServer:
    def __init__(self, cfg=DEFAULT_CFG):
        self.cfg = cfg
        rclpy.init()
        self.node = Node("ros_tool_server")
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.current_gait = "4-legged"  # default gait

        logger.info("ROS Tool Server has been started.")

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

    async def stop_moving(self) -> Tuple[bool, str]:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        self.twist_pub.publish(twist)

        logger.info("🤖 Robot STOPPED")
        return True, "Robot stopped successfully"

    async def move(self, vx: float, vy: float, wz: float, duration: float) -> Tuple[bool, str]:
        warnings = []

        # Validate velocity constraints (same as openai_bridge)
        if abs(vx) > self.cfg.move_cfg.vx_max:
            vx = self.cfg.move_cfg.vx_max if vx > 0 else -self.cfg.move_cfg.vx_max
            warnings.append(f"vx clamped to {vx} (max: ±{self.cfg.move_cfg.vx_max})")

        if abs(vy) > self.cfg.move_cfg.vy_max:
            vy = self.cfg.move_cfg.vy_max if vy > 0 else -self.cfg.move_cfg.vy_max
            warnings.append(f"vy clamped to {vy} (max: ±{self.cfg.move_cfg.vy_max})")

        if abs(wz) > self.cfg.move_cfg.wz_max:
            wz = self.cfg.move_cfg.wz_max if wz > 0 else -self.cfg.move_cfg.wz_max
            warnings.append(f"wz clamped to {wz} (max: ±{self.cfg.move_cfg.wz_max})")

        # Check if all velocities are below their thresholds
        if (
            abs(vx) < self.cfg.move_cfg.vx_threshold
            and abs(vy) < self.cfg.move_cfg.vy_threshold
            and abs(wz) < self.cfg.move_cfg.wz_threshold
        ):
            warnings.append("All velocities (vx, vy, wz) are below their movement thresholds, robot may not move")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)

        self.twist_pub.publish(twist)

        message = f"Robot moving with velocities vx={vx}, vy={vy}, wz={wz}"
        if warnings:
            message += f". Warnings: {'; '.join(warnings)}"

        logger.info(f"🤖 Robot MOVING - vx: {vx}, vy: {vy}, wz: {wz}")
        if warnings:
            logger.warning(f"🤖 Movement warnings: {'; '.join(warnings)}")

        return True, message, warnings

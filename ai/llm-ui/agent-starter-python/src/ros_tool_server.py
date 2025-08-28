from typing import Tuple, Optional, Any, Dict
import rclpy
from rclpy.node import Node
import logging
from controller_manager_msgs.srv import SwitchController
from dataclasses import dataclass
from geometry_msgs.msg import Twist
import asyncio
from abc import ABC, abstractmethod
import time

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

########### TODO: put command logic inside the command objects ###########


class Command(ABC):
    """Base class for all robot commands"""

    def __init__(self, name: str):
        self.name = name
        self.timestamp = time.time()

    @abstractmethod
    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        """Execute the command and return success status and message"""
        pass


class MoveCommand(Command):
    def __init__(self, vx: float, vy: float, wz: float):
        super().__init__("move")
        self.vx = vx
        self.vy = vy
        self.wz = wz

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        return await server._execute_move(self.vx, self.vy, self.wz)


class StopCommand(Command):
    def __init__(self):
        super().__init__("stop")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        return await server._execute_stop()


class WaitCommand(Command):
    def __init__(self, duration: float):
        super().__init__("wait")
        self.duration = duration

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        await asyncio.sleep(self.duration)
        return True, f"Waited for {self.duration} seconds"


class ActivateCommand(Command):
    def __init__(self):
        super().__init__("activate")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        return await server._execute_activate()


class DeactivateCommand(Command):
    def __init__(self):
        super().__init__("deactivate")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        return await server._execute_deactivate()


class RosToolServer:
    def __init__(self, cfg=DEFAULT_CFG):
        self.cfg = cfg
        rclpy.init()
        self.node = Node("ros_tool_server")
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.current_gait = "4-legged"  # default gait

        # Initialize command queue
        self.command_queue = asyncio.Queue()
        self.queue_running = False
        self.queue_task = None

        # Create twist publisher for movement commands
        self.twist_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

        logger.info("ROS Tool Server has been started.")

    async def start_queue_processor(self):
        """Start the background task that processes commands from the queue"""
        if not self.queue_running:
            self.queue_running = True
            self.queue_task = asyncio.create_task(self._process_command_queue())
            logger.info("Command queue processor started")

    async def stop_queue_processor(self):
        """Stop the command queue processor"""
        self.queue_running = False
        if self.queue_task:
            await self.queue_task
            logger.info("Command queue processor stopped")

    async def _process_command_queue(self):
        """Background task that processes commands from the queue sequentially"""
        while self.queue_running:
            try:
                # Wait for a command with timeout to allow checking queue_running
                command = await asyncio.wait_for(self.command_queue.get(), timeout=0.1)
                logger.info(f"Executing command: {command.name}")

                try:
                    success, message = await command.execute(self)
                    if success:
                        logger.info(f"Command {command.name} succeeded: {message}")
                    else:
                        logger.error(f"Command {command.name} failed: {message}")
                except Exception as e:
                    logger.error(f"Error executing command {command.name}: {e}")

            except asyncio.TimeoutError:
                # Timeout is expected, continue loop to check queue_running
                continue
            except Exception as e:
                logger.error(f"Unexpected error in command queue processor: {e}")

    async def add_command(self, command: Command) -> None:
        """Add a command to the queue"""
        await self.command_queue.put(command)
        logger.info(f"Added command {command.name} to queue")

    async def queue_move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> Tuple[bool, str]:
        """Queue a move_for_time operation as 3 separate commands"""
        # Add move command
        await self.add_command(MoveCommand(vx, vy, wz))

        # Add wait command
        await self.add_command(WaitCommand(duration))

        # Add stop command
        await self.add_command(StopCommand())

        return True, f"Queued move_for_time: vx={vx}, vy={vy}, wz={wz} for {duration}s"

    async def queue_activate(self):
        """Queue an activate command"""
        logger.info("Queueing activate command")
        await self.add_command(ActivateCommand())
        return True, "Activate command queued"

    async def queue_deactivate(self):
        """Queue a deactivate command"""
        logger.info("Queueing deactivate command")
        await self.add_command(DeactivateCommand())
        return True, "Deactivate command queued"

    async def _execute_deactivate(self) -> Tuple[bool, str]:
        """Internal method to execute deactivate command"""
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

    async def _execute_activate(self) -> Tuple[bool, str]:
        """Internal method to execute activate command"""
        req = SwitchController.Request()
        controller_name = CONTROLLER_NAME_MAP[self.current_gait]

        req.activate_controllers = [controller_name]
        req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
        req.strictness = 1

        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.done() and future.result().ok:
            logger.info("🤖 Robot ACTIVATED")
            return True, f"Robot activated successfully with {self.current_gait} gait"
        else:
            logger.error("❌ Failed to activate robot")
            return False, "Failed to activate robot - controller switch failed"

    async def _execute_stop(self) -> Tuple[bool, str]:
        """Internal method to execute stop command"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        self.twist_pub.publish(twist)

        logger.info("🤖 Robot STOPPED")
        return True, "Robot stopped successfully"

    async def _execute_move(self, vx: float, vy: float, wz: float) -> Tuple[bool, str]:
        """Internal method to execute move command"""
        warnings = []

        # Validate velocity constraints
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

        return True, message

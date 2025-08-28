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
        warnings = []

        # Validate velocity constraints
        vx = self.vx
        vy = self.vy
        wz = self.wz

        if abs(vx) > server.cfg.move_cfg.vx_max:
            vx = server.cfg.move_cfg.vx_max if vx > 0 else -server.cfg.move_cfg.vx_max
            warnings.append(f"vx clamped to {vx} (max: ±{server.cfg.move_cfg.vx_max})")

        if abs(vy) > server.cfg.move_cfg.vy_max:
            vy = server.cfg.move_cfg.vy_max if vy > 0 else -server.cfg.move_cfg.vy_max
            warnings.append(f"vy clamped to {vy} (max: ±{server.cfg.move_cfg.vy_max})")

        if abs(wz) > server.cfg.move_cfg.wz_max:
            wz = server.cfg.move_cfg.wz_max if wz > 0 else -server.cfg.move_cfg.wz_max
            warnings.append(f"wz clamped to {wz} (max: ±{server.cfg.move_cfg.wz_max})")

        # Check if all velocities are below their thresholds
        if (
            abs(vx) < server.cfg.move_cfg.vx_threshold
            and abs(vy) < server.cfg.move_cfg.vy_threshold
            and abs(wz) < server.cfg.move_cfg.wz_threshold
        ):
            warnings.append("All velocities (vx, vy, wz) are below their movement thresholds, robot may not move")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)

        server.twist_pub.publish(twist)

        message = f"Robot moving with velocities vx={vx}, vy={vy}, wz={wz}"
        if warnings:
            message += f". Warnings: {'; '.join(warnings)}"

        logger.info(f"🤖 Robot MOVING - vx: {vx}, vy: {vy}, wz: {wz}")
        if warnings:
            logger.warning(f"🤖 Movement warnings: {'; '.join(warnings)}")

        return True, message


class StopCommand(Command):
    def __init__(self):
        super().__init__("stop")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

        server.twist_pub.publish(twist)

        logger.info("🤖 Robot STOPPED")
        return True, "Robot stopped successfully"


class WaitCommand(Command):
    def __init__(self, duration: float):
        super().__init__("wait")
        self.duration = duration

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        await asyncio.sleep(self.duration)
        return True, f"Waited for {self.duration} seconds"


class MoveForTimeCommand(Command):
    def __init__(self, vx: float, vy: float, wz: float, duration: float):
        super().__init__("move_for_time")
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.duration = duration

    # TODO: Make the queue processor accept "CompositeCommand" so this can be simpler
    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        # Execute move command
        move_cmd = MoveCommand(self.vx, self.vy, self.wz)
        success, message = await move_cmd.execute(server)
        if not success:
            return False, f"Move failed: {message}"

        # Wait for the specified duration
        wait_cmd = WaitCommand(self.duration)
        await wait_cmd.execute(server)

        # Execute stop command
        stop_cmd = StopCommand()
        success, message = await stop_cmd.execute(server)
        if not success:
            return False, f"Stop failed: {message}"

        return True, f"Completed move_for_time: vx={self.vx}, vy={self.vy}, wz={self.wz} for {self.duration}s"


class ActivateCommand(Command):
    def __init__(self):
        super().__init__("activate")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        req = SwitchController.Request()
        controller_name = CONTROLLER_NAME_MAP[server.current_gait]

        req.activate_controllers = [controller_name]
        req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
        req.strictness = 1

        future = server.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(server.node, future, timeout_sec=2.0)

        if future.done() and future.result().ok:
            logger.info("🤖 Robot ACTIVATED")
            return True, f"Robot activated successfully with {server.current_gait} gait"
        else:
            logger.error("❌ Failed to activate robot")
            return False, "Failed to activate robot - controller switch failed"


class DeactivateCommand(Command):
    def __init__(self):
        super().__init__("deactivate")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        req = SwitchController.Request()
        req.activate_controllers = []
        req.deactivate_controllers = list(AVAILABLE_CONTROLLERS)
        req.strictness = 1

        future = server.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(server.node, future, timeout_sec=2.0)

        if future.done() and future.result().ok:
            logger.info("🤖 Robot DEACTIVATED")
            return True, "Robot deactivated successfully"
        else:
            logger.error("❌ Failed to deactivate robot")
            return False, "Failed to deactivate robot - controller switch failed"


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
        self.current_command_task = None

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

                # Create a cancellable task for the command execution
                self.current_command_task = asyncio.create_task(command.execute(self))
                
                try:
                    success, message = await self.current_command_task
                    if success:
                        logger.info(f"Command {command.name} succeeded: {message}")
                    else:
                        logger.error(f"Command {command.name} failed: {message}")
                except asyncio.CancelledError:
                    logger.info(f"Command {command.name} was cancelled")
                    # Ensure robot is stopped after cancellation
                    stop_cmd = StopCommand()
                    await stop_cmd.execute(self)
                    logger.info("Robot stopped after command cancellation")
                except Exception as e:
                    logger.error(f"Error executing command {command.name}: {e}")
                finally:
                    self.current_command_task = None

            except asyncio.TimeoutError:
                # Timeout is expected, continue loop to check queue_running
                continue
            except Exception as e:
                logger.error(f"Unexpected error in command queue processor: {e}")

    async def add_command(self, command: Command) -> None:
        """Add a command to the queue"""
        await self.command_queue.put(command)
        logger.debug(f"Added command {command.name} to queue")

    async def queue_move_for_time(self, vx: float, vy: float, wz: float, duration: float) -> Tuple[bool, str]:
        """Queue a move_for_time operation as a single command"""
        logger.info(f"Queueing move_for_time command: vx={vx}, vy={vy}, wz={wz}, duration={duration}")
        await self.add_command(MoveForTimeCommand(vx, vy, wz, duration))
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
    
    async def interrupt_and_stop(self) -> Tuple[bool, str]:
        """Interrupt current command and immediately stop the robot"""
        logger.info("Interrupting current command and stopping robot")
        
        # Cancel the currently executing command if any
        if self.current_command_task and not self.current_command_task.done():
            logger.info(f"Cancelling current command task")
            self.current_command_task.cancel()
            # The cancellation handler in _process_command_queue will stop the robot
            
            # Wait a moment for the cancellation to complete
            try:
                await asyncio.wait_for(self.current_command_task, timeout=0.5)
            except (asyncio.CancelledError, asyncio.TimeoutError):
                pass
        else:
            # No command running, just stop the robot directly
            stop_cmd = StopCommand()
            await stop_cmd.execute(self)
        
        return True, "Current command interrupted and robot stopped"
    
    async def clear_queue(self) -> Tuple[bool, str]:
        """Clear all pending commands from the queue"""
        count = 0
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
                count += 1
            except asyncio.QueueEmpty:
                break
        
        logger.info(f"Cleared {count} commands from queue")
        return True, f"Cleared {count} pending commands from queue"
    
    async def emergency_stop(self) -> Tuple[bool, str]:
        """Emergency stop: interrupt current command, stop robot, and clear queue"""
        logger.warning("EMERGENCY STOP initiated")
        
        # First interrupt and stop
        await self.interrupt_and_stop()
        
        # Then clear the queue
        await self.clear_queue()
        
        logger.warning("EMERGENCY STOP completed")
        return True, "Emergency stop executed: robot stopped and queue cleared"

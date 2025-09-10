import base64
import random
from typing import Tuple, Optional, Any, Dict
import threading
import queue
from tool_server_abc import ToolServer
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import logging
from controller_manager_msgs.srv import SwitchController
from dataclasses import dataclass, field
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import asyncio
from abc import ABC, abstractmethod
import time

logger = logging.getLogger("ros_tool_server")
AVAILABLE_CONTROLLERS = {
    "neural_controller",
    "neural_controller_three_legged",
    "forward_kp_controller",
    "forward_position_controller",
    "forward_kd_controller",
}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}

# Single animation controller name
ANIMATION_CONTROLLER_NAME = "animation_controller"

# Animation name mapping from friendly names to exact CSV base names (without .csv extension)
ANIMATION_NAMES = {
    "twerk": "twerk_recording_2025-09-04_16-14-51_0",
    "lie_sit_lie": "lie_sit_lie_recording_2025-09-03_12-44-08_0",
    "lie_down": "lie_sit_lie_recording_2025-09-03_12-44-08_0",
    "stand_sit_shake_sit_stand": "stand_sit_shake_sit_stand_recording_2025-09-03_12-47-18_0",
    "stand_sit_shake": "stand_sit_shake_sit_stand_recording_2025-09-03_12-47-18_0",
    "shake": "stand_sit_shake_sit_stand_recording_2025-09-03_12-47-18_0",
    "stand_sit_stand": "stand_sit_stand_recording_2025-09-03_12-46-36_0",
    "sit": "stand_sit_stand_recording_2025-09-03_12-46-36_0",
}


@dataclass
class MoveCfg:
    vx_threshold: float = 0.4  # m/s
    vy_threshold: float = 0.4  # m/s
    wz_threshold: float = 30.0  # deg/s
    vx_max: float = 0.75  # m/s
    vy_max: float = 0.5  # m/s
    wz_max: float = 120.0  # deg/s


@dataclass
class DefaultCfg:
    move_cfg: MoveCfg = field(default_factory=MoveCfg)


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


# TODO: The warnings will only show up on execution but the LLM won't see them and would benefit from seeing the warnings when queueing
class MoveCommand(Command):
    def __init__(self, vx: float, vy: float, wz: float, server: "RosToolServer"):
        super().__init__("move")
        self.vx = vx
        self.vy = vy
        self.wz = wz

        if abs(vx) > server.cfg.move_cfg.vx_max:
            raise ValueError(f"vx {vx} exceeds max limit of ±{server.cfg.move_cfg.vx_max}")
        if abs(vy) > server.cfg.move_cfg.vy_max:
            raise ValueError(f"vy {vy} exceeds max limit of ±{server.cfg.move_cfg.vy_max}")
        if abs(wz) > server.cfg.move_cfg.wz_max:
            raise ValueError(f"wz {wz} exceeds max limit of ±{server.cfg.move_cfg.wz_max}")

        # Check if all velocities are below their thresholds
        if (
            abs(vx) < server.cfg.move_cfg.vx_threshold
            and abs(vy) < server.cfg.move_cfg.vy_threshold
            and abs(wz) < server.cfg.move_cfg.wz_threshold
        ):
            raise ValueError("All velocities (vx, vy, wz) are below their movement thresholds, robot may not move")

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        # Validate velocity constraints
        vx = self.vx
        vy = self.vy
        wz = self.wz

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz) / 57.2958  # Convert deg/s to rad/s

        server.twist_pub.publish(twist)

        message = f"Robot moving with velocities vx={vx}, vy={vy}, wz={wz}"
        logger.info(f"Executing MoveCommand - vx: {vx}, vy: {vy}, wz: {wz}")
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

        logger.info("Executed StopCommand")
        return True, "Robot stopped successfully"


class WaitCommand(Command):
    def __init__(self, duration: float):
        super().__init__("wait")
        self.duration = duration

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        await asyncio.sleep(self.duration)
        return True, f"Waited for {self.duration} seconds"


class MoveForTimeCommand(Command):
    def __init__(self, vx: float, vy: float, wz: float, duration: float, server: "RosToolServer"):
        super().__init__("move_for_time")

        self.move_cmd = MoveCommand(vx, vy, wz, server)
        self.wait_cmd = WaitCommand(duration)
        self.stop_cmd = StopCommand()

    # TODO: Make the queue processor accept "CompositeCommand" so this can be simpler
    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        # Execute move command
        success, message = await self.move_cmd.execute(server)
        if not success:
            return False, f"Move failed: {message}"

        # Wait for the specified duration
        success, message = await self.wait_cmd.execute(server)

        # Execute stop command
        success, message = await self.stop_cmd.execute(server)
        if not success:
            return False, f"Stop failed: {message}"

        return (
            True,
            f"Completed move_for_time: vx={self.move_cmd.vx}, vy={self.move_cmd.vy}, wz={self.move_cmd.wz} for {self.wait_cmd.duration}s",
        )


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


class AnimationCommand(Command):
    def __init__(self, animation_name: str):
        super().__init__(f"animation_{animation_name}")
        self.animation_name = animation_name

        # Validate animation name and resolve alias
        if animation_name not in ANIMATION_NAMES:
            raise ValueError(
                f"Unknown animation '{animation_name}'. Available animations: {list(ANIMATION_NAMES.keys())}"
            )

        # Get the actual animation name (resolves aliases)
        self.actual_animation_name = ANIMATION_NAMES[animation_name]

    async def execute(self, server: "RosToolServer") -> Tuple[bool, str]:
        try:
            # First, switch to the animation controller
            req = SwitchController.Request()
            all_controllers = list(AVAILABLE_CONTROLLERS) + [ANIMATION_CONTROLLER_NAME]
            req.activate_controllers = [ANIMATION_CONTROLLER_NAME]
            req.deactivate_controllers = [c for c in all_controllers if c != ANIMATION_CONTROLLER_NAME]
            req.strictness = 1

            future = server.switch_controller_client.call_async(req)
            rclpy.spin_until_future_complete(server.node, future, timeout_sec=2.0)

            if not (future.done() and future.result().ok):
                logger.error(f"❌ Failed to switch to animation controller for animation '{self.animation_name}'")
                return False, f"Failed to switch to animation controller for animation '{self.animation_name}'"

            # Then, publish the animation name to the controller's topic
            topic_name = f"/{ANIMATION_CONTROLLER_NAME}/animation_select"
            if topic_name not in server.animation_publishers:
                # Create publisher if it doesn't exist
                server.animation_publishers[topic_name] = server.node.create_publisher(String, topic_name, 10)

            # Publish animation selection
            msg = String()
            msg.data = self.actual_animation_name
            server.animation_publishers[topic_name].publish(msg)

            logger.info(f"🎭 Animation '{self.animation_name}' (actual: '{self.actual_animation_name}') started")
            return True, f"Animation '{self.animation_name}' started successfully"

        except Exception as e:
            logger.error(f"❌ Failed to start animation '{self.animation_name}': {e}")
            return False, f"Failed to start animation '{self.animation_name}': {e}"


class RosToolServer(ToolServer):
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

        # Dictionary to hold animation publishers (created on demand)
        self.animation_publishers = {}

        # Image subscription node: keep only the most recent compressed image
        self.image_node = Node("ros_tool_server_images")
        self.latest_image_queue: "queue.Queue[CompressedImage]" = queue.Queue(maxsize=1)
        self.image_sub = self.image_node.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self._on_image,
            1,
        )

        # Start ROS executor in a background thread
        self.executor = SingleThreadedExecutor()
        # Only add the image node to this executor so control node remains
        # free to use spin_until_future_complete in service calls.
        self.executor.add_node(self.image_node)
        self._ros_thread = threading.Thread(target=self._spin_executor, name="ros-executor", daemon=True)
        self._ros_thread.start()

        self.start_queue_processor()

        logger.info("ROS Tool Server has been started.")

    async def get_camera_image(self, context: Any) -> Dict[str, Any]:
        from livekit.agents.llm import ImageContent

        latest_ros_compressed_img_msg = self.latest_image_queue.get_nowait()
        b64 = base64.b64encode(latest_ros_compressed_img_msg.data).decode("utf-8")
        ctx = context.session.current_agent.chat_ctx.copy()
        ctx.add_message(
            role="user",
            content=[ImageContent(image=f"data:image/jpeg;base64,{b64}")],
        )
        logger.info(f"Adding image to conversation, size: {len(b64)} bytes.....")
        await context.session.current_agent.update_chat_ctx(ctx)
        # log image number of bytes
        logger.info(f"Added image to conversation, size: {len(b64)} bytes")
        return True, "Image added to conversation"

    # TODO: LLM might want to start the queue processor explicitly so it can control when commands start executing.
    # For now, we can start it automatically when the server is created.
    # For instance, the LLM often takes several seconds to queue up commands and it's possible the behavior will be wrong if
    # the commands start executing before the LLM is done queueing them.
    def start_queue_processor(self):
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

    def _spin_executor(self):
        """Spin the ROS executor in a dedicated thread."""
        try:
            self.executor.spin()
        except Exception as e:
            logger.error(f"ROS executor stopped with error: {e}")
            raise e

    def _on_image(self, msg: CompressedImage):
        """Keep only the latest image in a thread-safe single-slot queue."""
        try:
            # Drop previous image if present to keep only the latest
            if self.latest_image_queue.full():
                try:
                    self.latest_image_queue.get_nowait()
                except queue.Empty:
                    pass
            self.latest_image_queue.put_nowait(msg)
            logging.info("Enqueued latest image")
        except Exception as e:
            logger.warning(f"Failed to enqueue latest image: {e}")

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

                ################## Handle cancellation ##################
                except asyncio.CancelledError:
                    logger.info(f"Command {command.name} was cancelled")
                    # Ensure robot is stopped after cancellation
                    stop_cmd = StopCommand()
                    await stop_cmd.execute(self)
                    logger.info("Robot stopped after command cancellation")

                ################### Handle other exceptions ###################
                except Exception as e:
                    logger.error(f"Error executing command {command.name}: {e}")
                finally:
                    self.current_command_task = None

            except asyncio.TimeoutError:
                # Timeout is expected, continue loop to check queue_running
                # Log every 10 seconds if the queue is empty
                if random.random() < 0.01:
                    logger.info("Command queue processor waiting for commands...")
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
        try:
            move_for_time_cmd = MoveForTimeCommand(vx, vy, wz, duration, self)
        except ValueError as e:
            logger.warning(f"Invalid parameters for move_for_time: {e}")
            return False, str(e)
        await self.add_command(move_for_time_cmd)
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

    async def queue_stop(self):
        """Queue a stop command"""
        logger.info("Queueing stop command")
        await self.add_command(StopCommand())
        return True, "Stop command queued"

    async def queue_wait(self, duration: float):
        """Queue a wait command"""
        logger.info(f"Queueing wait command for {duration} seconds")
        await self.add_command(WaitCommand(duration))
        return True, f"Wait command for {duration} seconds queued"

    async def queue_animation(self, animation_name: str):
        """Queue an animation command"""
        logger.info(f"Queueing animation command: {animation_name}")
        try:
            animation_cmd = AnimationCommand(animation_name)
            await self.add_command(animation_cmd)
            return True, f"Animation '{animation_name}' queued"
        except ValueError as e:
            logger.warning(f"Invalid animation name: {e}")
            return False, str(e)

    async def _interrupt_and_stop(self) -> Tuple[bool, str]:
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

        # Then clear the queue
        await self.clear_queue()

        # First interrupt and stop
        await self._interrupt_and_stop()

        logger.warning("EMERGENCY STOP completed")
        return True, "Emergency stop executed: robot stopped and queue cleared"

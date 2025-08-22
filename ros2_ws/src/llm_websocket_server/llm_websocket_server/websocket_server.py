#!/usr/bin/env python3
"""
WebSocket server for LLM robot control integration.
Combines the robot_server.py WebSocket framework with ROS2 functionality
from openai_bridge for real robot control.
"""

import asyncio
import json
import logging
import websockets
import os
import subprocess
from datetime import datetime
from functools import partial
from typing import Union

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# Available controllers and mapping from openai_bridge
AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}

PORT = 8008


class RobotState:
    def __init__(self):
        self.is_active = False
        self.last_command = None
        self.command_count = 0
        self.current_gait = "4-legged"


class WebSocketRobotServer(Node):
    def __init__(self):
        super().__init__("llm_websocket_server")

        # Initialize robot state
        self.robot_state = RobotState()

        # Create ROS2 publishers and clients
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        # Wait for controller manager service to be available
        self.get_logger().info("Waiting for controller manager service...")
        self.switch_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Controller manager service available")

    async def handle_client(self, websocket):
        """Handle incoming WebSocket connections and messages."""
        client_addr = websocket.remote_address
        logger.info(f"🔌 New client connected: {client_addr}")

        try:
            async for message in websocket:
                try:
                    # Parse incoming message
                    data = json.loads(message)
                    command_name = data.get("name", "unknown")
                    command_args = data.get("args", {})

                    self.robot_state.command_count += 1
                    self.robot_state.last_command = command_name

                    logger.info(f"📨 Received command from {client_addr}: {command_name}")

                    # Handle different commands
                    response = await self.handle_command(command_name, command_args)

                    # Send response back to client
                    await websocket.send(json.dumps(response))
                    logger.info(f"📤 Sent response to {client_addr}: {response}")

                except json.JSONDecodeError:
                    error_response = {
                        "status": "error",
                        "message": "Invalid JSON format",
                        "timestamp": datetime.now().isoformat(),
                    }
                    await websocket.send(json.dumps(error_response))
                    logger.error(f"❌ Invalid JSON received from {client_addr}: {message}")

                except Exception as e:
                    error_response = {
                        "status": "error",
                        "message": f"Server error: {str(e)}",
                        "timestamp": datetime.now().isoformat(),
                    }
                    await websocket.send(json.dumps(error_response))
                    logger.error(f"❌ Error handling message from {client_addr}: {e}")

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🔌 Client disconnected: {client_addr}")
        except Exception as e:
            logger.error(f"❌ Connection error with {client_addr}: {e}")

    async def handle_command(self, command_name: str, command_args: dict) -> dict:
        """Handle specific robot commands and return appropriate responses."""

        if command_name == "activate":
            success, message = await self.activate_robot()
            return {
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

        elif command_name == "deactivate":
            success, message = await self.deactivate_robot()
            return {
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

        elif command_name == "move":
            vx = command_args.get("vx", 0.0)
            vy = command_args.get("vy", 0.0)
            wz = command_args.get("wz", 0.0)

            success, message, warnings = await self.move_robot(vx, vy, wz)

            response = {
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "velocities": {"vx": vx, "vy": vy, "wz": wz},
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

            if warnings:
                response["warnings"] = warnings
                response["constraints"] = {"max_vx": 0.75, "max_vy": 0.5, "max_wz": 2.0, "min_movement_threshold": 0.2}

            return response

        elif command_name == "get_battery":
            battery_percentage, battery_voltage = await self.get_battery_info()

            battery_status = "normal"
            if battery_percentage <= 15:
                battery_status = "critical"
            elif battery_percentage <= 30:
                battery_status = "low"

            logger.info(f"🔋 Battery level: {battery_percentage}% ({battery_voltage}V) ({battery_status})")
            return {
                "status": "success",
                "message": f"Battery at {battery_percentage}% ({battery_voltage}V)",
                "battery_percentage": battery_percentage,
                "battery_voltage": battery_voltage,
                "battery_status": battery_status,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

        elif command_name == "status":
            battery_percentage, battery_voltage = await self.get_battery_info()
            return {
                "status": "success",
                "message": "Status retrieved",
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "battery_percentage": battery_percentage,
                "battery_voltage": battery_voltage,
                "current_gait": self.robot_state.current_gait,
                "last_command": self.robot_state.last_command,
                "command_count": self.robot_state.command_count,
                "timestamp": datetime.now().isoformat(),
            }

        else:
            return {
                "status": "error",
                "message": f"Unknown command: {command_name}",
                "available_commands": ["activate", "deactivate", "move", "get_battery", "status"],
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

    async def activate_robot(self) -> tuple[bool, str]:
        """Activate the robot by switching to the default controller."""
        try:
            # Use the same logic as openai_bridge activate function
            req = SwitchController.Request()
            controller_name = CONTROLLER_NAME_MAP[self.robot_state.current_gait]

            req.activate_controllers = [controller_name]
            req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
            req.strictness = 1

            # Call the service synchronously in the async context
            future = self.switch_controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done() and future.result().ok:
                self.robot_state.is_active = True
                logger.info("🤖 Robot ACTIVATED")
                return True, f"Robot activated successfully with {self.robot_state.current_gait} gait"
            else:
                logger.error("❌ Failed to activate robot")
                return False, "Failed to activate robot - controller switch failed"

        except Exception as e:
            logger.error(f"❌ Error activating robot: {e}")
            return False, f"Failed to activate robot: {str(e)}"

    async def deactivate_robot(self) -> tuple[bool, str]:
        """Deactivate the robot by stopping all controllers."""
        try:
            req = SwitchController.Request()
            req.activate_controllers = []
            req.deactivate_controllers = list(AVAILABLE_CONTROLLERS)
            req.strictness = 1

            future = self.switch_controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done() and future.result().ok:
                self.robot_state.is_active = False
                logger.info("🤖 Robot DEACTIVATED")
                return True, "Robot deactivated successfully"
            else:
                logger.error("❌ Failed to deactivate robot")
                return False, "Failed to deactivate robot - controller switch failed"

        except Exception as e:
            logger.error(f"❌ Error deactivating robot: {e}")
            return False, f"Failed to deactivate robot: {str(e)}"

    async def move_robot(self, vx: float, vy: float, wz: float) -> tuple[bool, str, list]:
        """Move the robot with the specified velocities."""
        try:
            warnings = []

            # Validate velocity constraints (same as openai_bridge)
            if abs(vx) > 0.75:
                vx = 0.75 if vx > 0 else -0.75
                warnings.append(f"vx clamped to {vx} (max: ±0.75)")

            if abs(vy) > 0.5:
                vy = 0.5 if vy > 0 else -0.5
                warnings.append(f"vy clamped to {vy} (max: ±0.5)")

            if abs(wz) > 2.0:
                wz = 2.0 if wz > 0 else -2.0
                warnings.append(f"wz clamped to {wz} (max: ±2.0)")

            # Check minimum movement thresholds
            if 0 < abs(vx) < 0.2:
                warnings.append(f"vx={vx} is below movement threshold (0.2), robot may not move")
            if 0 < abs(vy) < 0.2:
                warnings.append(f"vy={vy} is below movement threshold (0.2), robot may not move")

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

        except Exception as e:
            logger.error(f"❌ Error moving robot: {e}")
            return False, f"Failed to move robot: {str(e)}", []

    async def get_battery_info(self) -> tuple[int, float]:
        """Get battery percentage and voltage using the existing battery check script."""
        try:
            # Use the same battery check script as openai_bridge
            result = subprocess.run(
                ["python3", "/home/pi/pupperv3-monorepo/robot/utils/check_batt_voltage.py"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )

            if result.returncode == 0:
                # Parse the output: "Battery:	85%	16.5V"
                output = result.stdout.strip()
                if "Battery:" in output:
                    parts = output.split("\t")
                    if len(parts) >= 3:
                        percentage_str = parts[1].replace("%", "")
                        voltage_str = parts[2].replace("V", "")
                        percentage = int(percentage_str)
                        voltage = float(voltage_str)
                        return percentage, voltage

            # Fallback to mock values if script fails
            logger.warning("🔋 Battery script failed, using fallback values")
            return 0, 0

        except Exception as e:
            logger.error(f"❌ Error getting battery info: {e}")
            return 0, 0  # Fallback values

    async def start_websocket_server(self):
        """Start the WebSocket server."""
        host = "localhost"
        port = PORT

        logger.info(f"🚀 Starting LLM WebSocket server on {host}:{port}")
        logger.info(f"🤖 Initial robot state: {'ACTIVE' if self.robot_state.is_active else 'INACTIVE'}")
        logger.info("📡 Waiting for connections...")

        try:
            async with websockets.serve(self.handle_client, host, port):
                await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("🛑 Server stopped by user")
        except Exception as e:
            logger.error(f"❌ Server error: {e}")


def main():
    """Main entry point."""
    rclpy.init()

    try:
        server_node = WebSocketRobotServer()

        # Run the WebSocket server in an async event loop
        asyncio.run(server_node.start_websocket_server())

    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

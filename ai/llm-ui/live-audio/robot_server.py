#!/usr/bin/env python3
"""
Simple WebSocket server to test robot control functionality.
This is a mockup that responds to activate/deactivate commands.
"""

import asyncio
import json
import logging
import websockets
import psutil
import platform
import math
import time
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


class RobotState:
    def __init__(self):
        self.is_active = False
        self.last_command = None
        self.command_count = 0
        self.start_time = time.time()

    def get_battery_percentage(self):
        """Generate sinusoidal battery percentage at 10Hz frequency."""
        # 10Hz means 10 complete cycles per second
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Generate sine wave: 10Hz frequency, oscillate between 20% and 90%
        # sin() oscillates between -1 and 1, we map to 20-90%
        sine_value = math.sin(2 * math.pi * 0.15 * elapsed_time)
        battery_percentage = 55 + 35 * sine_value  # Center at 55%, ±35% range (20-90%)

        return int(battery_percentage)


robot_state = RobotState()


def add_request_id(response: dict, request_id: str = None) -> dict:
    """Add request_id to response if provided."""
    if request_id:
        response["request_id"] = request_id
    return response


async def handle_client(websocket):
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
                request_id = data.get("request_id")

                robot_state.command_count += 1
                robot_state.last_command = command_name

                logger.info(f"📨 Received command from {client_addr}: {command_name}")

                # Handle different commands
                response = await handle_command(command_name, command_args, request_id)

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


async def handle_command(command_name: str, command_args: dict, request_id: str = None) -> dict:
    """Handle specific robot commands and return appropriate responses."""

    if command_name == "activate":
        if robot_state.is_active:
            return add_request_id(
                {
                    "status": "warning",
                    "message": "Robot is already active",
                    "robot_state": "active",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )
        else:
            robot_state.is_active = True
            logger.info("🤖 Robot ACTIVATED")
            return add_request_id(
                {
                    "status": "success",
                    "message": "Robot activated successfully",
                    "robot_state": "active",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )

    elif command_name == "deactivate":
        if not robot_state.is_active:
            return add_request_id(
                {
                    "status": "warning",
                    "message": "Robot is already inactive",
                    "robot_state": "inactive",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )
        else:
            robot_state.is_active = False
            logger.info("🤖 Robot DEACTIVATED")
            return add_request_id(
                {
                    "status": "success",
                    "message": "Robot deactivated successfully",
                    "robot_state": "inactive",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )

    elif command_name == "move":
        vx = command_args.get("vx", 0.0)
        vy = command_args.get("vy", 0.0)
        wz = command_args.get("wz", 0.0)

        if not robot_state.is_active:
            return add_request_id(
                {
                    "status": "warning",
                    "message": "Cannot move - robot is not active. Activate robot first.",
                    "robot_state": "inactive",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )

        # Validate velocity constraints
        warnings = []

        # Check maximum limits
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

        message = f"Robot moving with velocities vx={vx}, vy={vy}, wz={wz}"
        if warnings:
            message += f". Warnings: {'; '.join(warnings)}"

        logger.info(f"🤖 Robot MOVING - vx: {vx}, vy: {vy}, wz: {wz}")
        if warnings:
            logger.warning(f"🤖 Movement warnings: {'; '.join(warnings)}")

        return add_request_id(
            {
                "status": "success",
                "message": message,
                "robot_state": "active",
                "velocities": {"vx": vx, "vy": vy, "wz": wz},
                "warnings": warnings if warnings else None,
                "constraints": {"max_vx": 0.75, "max_vy": 0.5, "max_wz": 2.0, "min_movement_threshold": 0.2},
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count,
            },
            request_id,
        )

    elif command_name == "get_battery":
        # Get dynamic sinusoidal battery percentage
        battery_percentage = robot_state.get_battery_percentage()

        battery_status = "normal"
        if battery_percentage <= 15:
            battery_status = "critical"
        elif battery_percentage <= 30:
            battery_status = "low"

        logger.info(f"🔋 Battery level: {battery_percentage}% ({battery_status})")
        return add_request_id(
            {
                "status": "success",
                "message": f"Battery at {battery_percentage}%",
                "battery_percentage": battery_percentage,
                "battery_status": battery_status,
                "robot_state": "active" if robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count,
            },
            request_id,
        )

    elif command_name == "get_cpu_usage":
        try:
            # Get CPU usage (instant measurement, no blocking)
            cpu_usage = psutil.cpu_percent()
            system_info = {
                "platform": platform.system(),
                "cpu_count": psutil.cpu_count(),
                "cpu_count_logical": psutil.cpu_count(logical=True),
            }

            logger.info(f"💻 CPU usage: {cpu_usage}%")
            return add_request_id(
                {
                    "status": "success",
                    "message": f"CPU usage at {cpu_usage}%",
                    "cpu_usage": round(cpu_usage, 1),
                    "system_info": system_info,
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )
        except Exception as e:
            logger.error(f"❌ Failed to get CPU usage: {e}")
            return add_request_id(
                {
                    "status": "error",
                    "message": f"Failed to get CPU usage: {str(e)}",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": robot_state.command_count,
                },
                request_id,
            )

    elif command_name == "status":
        battery_percentage = robot_state.get_battery_percentage()
        return add_request_id(
            {
                "status": "success",
                "message": "Status retrieved",
                "robot_state": "active" if robot_state.is_active else "inactive",
                "battery_percentage": battery_percentage,
                "last_command": robot_state.last_command,
                "command_count": robot_state.command_count,
                "timestamp": datetime.now().isoformat(),
            },
            request_id,
        )

    else:
        return add_request_id(
            {
                "status": "error",
                "message": f"Unknown command: {command_name}",
                "available_commands": ["activate", "deactivate", "move", "get_battery", "get_cpu_usage", "status"],
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count,
            },
            request_id,
        )


async def main():
    """Start the WebSocket server."""
    host = "localhost"
    port = 8008

    logger.info(f"🚀 Starting robot WebSocket server on {host}:{port}")
    logger.info(f"🤖 Initial robot state: {'ACTIVE' if robot_state.is_active else 'INACTIVE'}")
    logger.info("📡 Waiting for connections...")

    try:
        async with websockets.serve(handle_client, host, port):
            await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        logger.info("🛑 Server stopped by user")
    except Exception as e:
        logger.error(f"❌ Server error: {e}")


if __name__ == "__main__":
    asyncio.run(main())

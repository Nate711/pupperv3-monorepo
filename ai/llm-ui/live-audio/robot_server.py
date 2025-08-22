#!/usr/bin/env python3
"""
Simple WebSocket server to test robot control functionality.
This is a mockup that responds to activate/deactivate commands.
"""

import asyncio
import json
import logging
import websockets
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RobotState:
    def __init__(self):
        self.is_active = False
        self.last_command = None
        self.command_count = 0

robot_state = RobotState()

async def handle_client(websocket):
    """Handle incoming WebSocket connections and messages."""
    client_addr = websocket.remote_address
    logger.info(f"🔌 New client connected: {client_addr}")
    
    try:
        async for message in websocket:
            try:
                # Parse incoming message
                data = json.loads(message)
                command_name = data.get('name', 'unknown')
                command_args = data.get('args', {})
                
                robot_state.command_count += 1
                robot_state.last_command = command_name
                
                logger.info(f"📨 Received command from {client_addr}: {command_name}")
                
                # Handle different commands
                response = await handle_command(command_name, command_args)
                
                # Send response back to client
                await websocket.send(json.dumps(response))
                logger.info(f"📤 Sent response to {client_addr}: {response}")
                
            except json.JSONDecodeError:
                error_response = {
                    "status": "error",
                    "message": "Invalid JSON format",
                    "timestamp": datetime.now().isoformat()
                }
                await websocket.send(json.dumps(error_response))
                logger.error(f"❌ Invalid JSON received from {client_addr}: {message}")
                
            except Exception as e:
                error_response = {
                    "status": "error", 
                    "message": f"Server error: {str(e)}",
                    "timestamp": datetime.now().isoformat()
                }
                await websocket.send(json.dumps(error_response))
                logger.error(f"❌ Error handling message from {client_addr}: {e}")
                
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"🔌 Client disconnected: {client_addr}")
    except Exception as e:
        logger.error(f"❌ Connection error with {client_addr}: {e}")

async def handle_command(command_name: str, command_args: dict) -> dict:
    """Handle specific robot commands and return appropriate responses."""
    
    if command_name == "activate":
        if robot_state.is_active:
            return {
                "status": "warning",
                "message": "Robot is already active",
                "robot_state": "active",
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count
            }
        else:
            robot_state.is_active = True
            logger.info("🤖 Robot ACTIVATED")
            return {
                "status": "success",
                "message": "Robot activated successfully", 
                "robot_state": "active",
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count
            }
    
    elif command_name == "deactivate":
        if not robot_state.is_active:
            return {
                "status": "warning",
                "message": "Robot is already inactive",
                "robot_state": "inactive", 
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count
            }
        else:
            robot_state.is_active = False
            logger.info("🤖 Robot DEACTIVATED")
            return {
                "status": "success",
                "message": "Robot deactivated successfully",
                "robot_state": "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": robot_state.command_count
            }
    
    elif command_name == "status":
        return {
            "status": "success",
            "message": "Status retrieved",
            "robot_state": "active" if robot_state.is_active else "inactive",
            "last_command": robot_state.last_command,
            "command_count": robot_state.command_count,
            "timestamp": datetime.now().isoformat()
        }
    
    else:
        return {
            "status": "error",
            "message": f"Unknown command: {command_name}",
            "available_commands": ["activate", "deactivate", "status"],
            "timestamp": datetime.now().isoformat(),
            "command_count": robot_state.command_count
        }

async def main():
    """Start the WebSocket server."""
    host = "localhost"
    port = 8765
    
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
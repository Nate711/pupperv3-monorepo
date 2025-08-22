# LLM WebSocket Server

A ROS2 package that provides a WebSocket server for LLM-based robot control. This package combines the WebSocket framework from `robot_server.py` with real ROS2 functionality from `openai_bridge` to enable voice control of the Pupper robot through the live-audio web application.

## Features

- **WebSocket Server**: Listens on `localhost:8765` for commands from the live-audio web app
- **ROS2 Integration**: Real robot control using `/cmd_vel` topic and controller manager services
- **Robot Commands**:
  - `activate`: Start the robot controllers
  - `deactivate`: Stop all robot controllers  
  - `move`: Move with velocity constraints (vx, vy, wz)
  - `get_battery`: Get real battery percentage and voltage
  - `status`: Get comprehensive robot status

## Installation

1. Build the package:
```bash
cd /path/to/ros2_ws
colcon build --packages-select llm_websocket_server
source install/setup.bash
```

2. Install Python dependencies:
```bash
pip install websockets
```

## Usage

### Start the WebSocket Server

```bash
ros2 run llm_websocket_server websocket_server
```

The server will:
- Initialize ROS2 node and wait for controller manager service
- Start WebSocket server on `localhost:8765`
- Handle commands from the live-audio web application

### Test with Live-Audio App

1. Start the WebSocket server:
```bash
ros2 run llm_websocket_server websocket_server
```

2. Launch the live-audio web app:
```bash
cd /path/to/ai/llm-ui/live-audio
npm run dev:http  # or npm run dev for HTTPS
```

3. Use voice commands:
- "Activate the robot"
- "Move forward"
- "Turn left"
- "Check battery level"
- "Deactivate robot"

## Message Format

### Request Format
```json
{
  "name": "move",
  "args": {
    "vx": 0.5,
    "vy": 0.0,
    "wz": 0.0
  }
}
```

### Response Format
```json
{
  "status": "success",
  "message": "Robot moving with velocities vx=0.5, vy=0.0, wz=0.0",
  "robot_state": "active",
  "velocities": {"vx": 0.5, "vy": 0.0, "wz": 0.0},
  "timestamp": "2024-08-22T11:30:00.123456",
  "command_count": 42
}
```

## Velocity Constraints

- **vx (forward/backward)**: ±0.75 m/s max, ≥0.2 m/s minimum for movement
- **vy (left/right)**: ±0.5 m/s max, ≥0.2 m/s minimum for movement  
- **wz (rotation)**: ±2.0 rad/s max

## Dependencies

- **ROS2 Packages**: `geometry_msgs`, `controller_manager_msgs`
- **Python**: `rclpy`, `websockets`, `asyncio`
- **Hardware**: Access to battery monitoring script at `/robot/utils/check_batt_voltage.py`

## Architecture

```
Live-Audio Web App (WebSocket Client)
    ↓ JSON Messages
WebSocket Server (This Package)
    ↓ ROS2 Topics/Services
Robot Controllers & Hardware
```

## Differences from robot_server.py

- **Real ROS2 Integration**: Uses actual `/cmd_vel` topic and controller services
- **Real Battery Monitoring**: Calls hardware battery script instead of mock values
- **Controller Management**: Handles neural controller activation/deactivation
- **Production Ready**: Proper error handling and logging for robot deployment
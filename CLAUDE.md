# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the Pupper V3 monorepo - a quadruped robot with ROS2 control, neural network locomotion policies, and an AI voice assistant interface.

## Build Commands

### ROS2 Workspace
```bash
# Build the ROS2 workspace
cd ros2_ws
./build.sh

# Or manually:
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Python Projects (using uv)
```bash
# AI agent
cd ai/llm-ui/agent-starter-python
uv sync

# Moonshine audio
cd ai/llm-ui/moonshine-test
uv sync

# RL training
cd ai/rl
uv sync
```

### TypeScript UI
```bash
cd ai/llm-ui/live-audio
npm install
npm run dev  # Development server
npm run build  # Production build
```

### Rust GUI
```bash
cd pupper-rs
cargo build  # Local build
just build   # Build for both local and ARM64 target
just sync    # Deploy to robot
```

## Run Commands

### Robot Operation
```bash
# Real robot
ros2 launch neural_controller launch.py

# Simulator
ros2 launch neural_controller launch.py sim:=True

# With teleop
ros2 launch neural_controller launch.py teleop:=True
```

### Voice Assistant UI
```bash
# Start complete UI stack (WebSocket server + web UI)
./robot/start_ui.sh

# Or manually:
# Terminal 1: WebSocket server
cd ros2_ws/src/llm_websocket_server
python3 llm_websocket_server/websocket_server.py

# Terminal 2: Web UI
cd ai/llm-ui/live-audio
npm run dev
```

## Testing

### Python Testing
```bash
# Agent tests
cd ai/llm-ui/agent-starter-python
pytest

# Run linting
ruff check src/
ruff format src/
```

### ROS2 Testing
```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

## Architecture

### System Components

1. **ROS2 Control Stack** (`ros2_ws/src/`)
   - `control_board_hardware_interface`: Hardware abstraction for motor control and IMU
   - `neural_controller`: MLP policy for locomotion, trained in simulation
   - `pupper_v3_description`: URDF/Xacro robot description files
   - `pupperv3_mujoco_sim`: MuJoCo simulation interface
   - `real2sim_controller`: System identification and sim-to-real transfer

2. **AI/ML Stack** (`ai/`)
   - `rl/`: JAX/Brax-based reinforcement learning for locomotion policies
   - `llm-ui/agent-starter-python`: LiveKit-based voice assistant using OpenAI/Cartesia
   - `llm-ui/live-audio`: TypeScript web interface with real-time audio streaming
   - `llm-ui/moonshine`: On-device speech recognition

3. **Robot Services** (`robot/`)
   - System services for battery monitoring, auto-start
   - tmux-based UI launcher scripts

### Key Interfaces

- **ROS2 Topics**: The neural controller subscribes to `/cmd_vel` for velocity commands and publishes joint states
- **WebSocket**: Real-time bidirectional communication between web UI and ROS2 backend (port 8765)
- **LiveKit**: WebRTC-based real-time audio/video streaming for voice assistant

### Policy Training Workflow

1. Train policies in `ai/rl/` using MJX simulation
2. Export trained policy to `ros2_ws/src/neural_controller/launch/policy.json`
3. Neural controller loads and executes policy in real-time at 200Hz

### Hardware Configuration

- **Motors**: 12 servos with position control via SPI
- **IMU**: BNO055 for orientation sensing
- **Compute**: Raspberry Pi 5 (robot) or desktop (development)
- **Audio**: USB microphone array + speaker for voice interaction
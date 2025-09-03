# Animation Controller

A ROS 2 controller that reads joint position keyframes from CSV files and plays them back at configurable frame rates. Designed for the Pupper V3 quadruped robot's 12-joint system.

## Features

- **CSV-based Animation**: Load keyframes from CSV files where each row is a keyframe and each column is a joint position
- **Configurable Frame Rate**: Set playback speed in Hz
- **Smooth Interpolation**: Optional interpolation between keyframes for smooth motion
- **Looping**: Option to loop animations continuously
- **Service Control**: Start/stop/reset animation via ROS services
- **Real-time Publishing**: Publishes current animation state and joint commands
- **Smooth Startup**: Gradually transitions from current position to first animation frame

## Package Structure

```
animation_controller/
├── CMakeLists.txt
├── package.xml
├── animation_controller.xml          # Plugin export
├── README.md
├── include/animation_controller/
│   └── animation_controller.hpp      # Main header file
├── src/
│   ├── animation_controller.cpp      # Implementation
│   └── animation_controller_parameters.yaml  # Parameter definitions
└── launch/
    ├── animation_playback.launch.py  # Main launch file
    ├── animation_config.yaml         # Default configuration
    └── example_animation.csv         # Example CSV file
```

## CSV File Format

Each row represents a keyframe, each column represents a joint position in radians.

**Joint order (12 joints total):**
1. `front_right_hip_joint`
2. `front_right_upper_leg_joint`
3. `front_right_lower_leg_joint`
4. `front_left_hip_joint`
5. `front_left_upper_leg_joint`
6. `front_left_lower_leg_joint`
7. `rear_right_hip_joint`
8. `rear_right_upper_leg_joint`
9. `rear_right_lower_leg_joint`
10. `rear_left_hip_joint`
11. `rear_left_upper_leg_joint`
12. `rear_left_lower_leg_joint`

### Example CSV:
```csv
# Standing neutral position
0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0
# Sit preparation
0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6
# Sitting position
0.0, 1.2, -2.4, 0.0, 1.2, -2.4, 0.0, 1.2, -2.4, 0.0, 1.2, -2.4
```

## Usage

### Building

```bash
cd ros2_ws
colcon build --packages-select animation_controller
```

### Basic Launch

```bash
# Basic usage with CSV file
ros2 launch animation_controller animation_playback.launch.py \
  csv_file:=/path/to/your/animation.csv

# With custom frame rate and looping
ros2 launch animation_controller animation_playback.launch.py \
  csv_file:=/path/to/your/animation.csv \
  frame_rate:=60.0 \
  loop:=true \
  auto_start:=true

# In simulation
ros2 launch animation_controller animation_playback.launch.py \
  sim:=true \
  csv_file:=/path/to/your/animation.csv
```

### Playback Control

Control animation playback using ROS services:

```bash
# Start animation playback
ros2 service call /animation_controller/play_animation std_srvs/srv/Empty

# Stop animation playback
ros2 service call /animation_controller/stop_animation std_srvs/srv/Empty

# Reset animation to beginning
ros2 service call /animation_controller/reset_animation std_srvs/srv/Empty
```

### Monitoring Animation State

```bash
# Monitor current animation state
ros2 topic echo /animation_controller/animation_state
```

The `animation_state` topic publishes a `Float32MultiArray` containing:
- **Elements [0-11]**: Current target positions for all 12 joints
- **Element [12]**: Current frame index
- **Element [13]**: Total number of frames in animation  
- **Element [14]**: Animation active status (1.0 = playing, 0.0 = stopped)

## Configuration

Key parameters in `animation_config.yaml`:

### Animation Settings
- `csv_file_path`: Path to your CSV animation file
- `frame_rate`: Playback speed in Hz (default: 30.0)
- `interpolation_enabled`: Smooth interpolation between keyframes (default: true)
- `loop_animation`: Loop continuously (default: false)
- `auto_start`: Start automatically when activated (default: false)

### Control Parameters
- `kps`: Position gains for all joints (array of 12 values)
- `kds`: Velocity gains for all joints (array of 12 values)
- `init_kps`: Gains during initialization phase (array of 12 values)
- `init_kds`: Gains during initialization phase (array of 12 values)

### Timing
- `init_duration`: Time to smoothly move from current position to first frame (default: 2.0s)
- `estop_kd`: Emergency stop damping value (default: 0.1)

## Controller Lifecycle

1. **Initialization** (`on_init`):
   - Loads and validates CSV file
   - Checks parameter consistency
   - Validates frame rate and joint count

2. **Activation** (`on_activate`):
   - Maps hardware interfaces for all 12 joints
   - Reads current joint positions
   - Transitions smoothly to first animation frame over `init_duration`
   - Sets up control services and state publisher
   - Initializes animation state

3. **Operation** (`update`):
   - Handles initialization phase with smooth transition
   - Updates animation time and calculates current frame
   - Performs interpolation between keyframes if enabled
   - Commands joint positions with specified gains
   - Publishes animation state at controller frequency (200 Hz)
   - Handles looping and animation completion

4. **Deactivation** (`on_deactivate`):
   - Safely stops all motion
   - Sets high damping on all joints
   - Releases hardware interfaces

## Integration

The controller is registered as `animation_controller/AnimationController` in the ROS 2 control framework. It can be used alongside other controllers by switching between them using the controller manager.

### Example Integration with Neural Controller

You can create launch files that allow switching between neural control and animation playback:

```bash
# Start with neural controller
ros2 launch neural_controller launch.py

# Switch to animation controller
ros2 control switch_controller \
  --deactivate neural_controller \
  --activate animation_controller

# Switch back to neural controller  
ros2 control switch_controller \
  --deactivate animation_controller \
  --activate neural_controller
```

## Error Handling

The controller includes robust error handling:
- **CSV Parse Errors**: Detailed error messages with line/column information
- **Missing Files**: Clear error messages for file access issues
- **Parameter Validation**: Checks for correct joint count and parameter sizes
- **Hardware Interface**: Graceful handling of missing or invalid interfaces
- **Animation Bounds**: Safe clamping of frame indices and interpolation values

## Development

### Adding New Features

1. **New Parameters**: Add to `animation_controller_parameters.yaml`
2. **CSV Extensions**: Modify `load_animation_csv()` method
3. **Playback Modes**: Extend the `update()` method logic
4. **Additional Services**: Add service callbacks in `on_activate()`

### Testing

Create test CSV files and use the provided example:

```bash
# Test with example animation
ros2 launch animation_controller animation_playback.launch.py \
  csv_file:=$(ros2 pkg prefix animation_controller)/share/animation_controller/launch/example_animation.csv \
  auto_start:=true
```

## Dependencies

- `controller_interface`: Base controller framework
- `hardware_interface`: Hardware abstraction layer
- `rclcpp_lifecycle`: Lifecycle node support
- `realtime_tools`: Real-time safe utilities
- `std_srvs`: Standard service definitions
- `generate_parameter_library`: Parameter validation
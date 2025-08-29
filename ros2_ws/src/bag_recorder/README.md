# Bag Recorder

A ROS2 node that allows recording MCAP bags using joystick buttons, while filtering out raw image topics.

## Features

- Start/stop recording using configurable joystick buttons
- Records all topics except raw image topics (to save space)
- Saves recordings as MCAP format files with timestamps
- Configurable output directory
- Thread-safe recording control

## Default Button Mapping

- **L1 (Button 6)**: Start recording
- **R1 (Button 7)**: Stop recording

## Usage

### Build the package

```bash
cd /path/to/ros2_ws
colcon build --packages-select bag_recorder
source install/setup.bash
```

### Run the node

```bash
# Basic usage
ros2 run bag_recorder bag_recorder_node

# Or with launch file
ros2 launch bag_recorder bag_recorder.launch.py

# Custom button mapping
ros2 launch bag_recorder bag_recorder.launch.py record_start_button:=4 record_stop_button:=5

# Custom output directory
ros2 launch bag_recorder bag_recorder.launch.py bag_output_dir:=/custom/path/bags
```

## Parameters

- `record_start_button` (int, default: 6): Joystick button index to start recording
- `record_stop_button` (int, default: 7): Joystick button index to stop recording  
- `bag_output_dir` (string, default: "~/bags"): Directory to save bag files

## Filtered Topics

The recorder excludes topics that typically contain raw image data to save storage space:

- Topics containing `image_raw`
- Topics containing `camera/image_raw` 
- Topics containing `rgb/image_raw`
- Topics containing `depth/image_raw`

## Output Format

Recordings are saved as MCAP files with timestamp-based filenames:
```
recording_2024-01-15_14-30-25.mcap
```

## Requirements

- ROS2 (Humble or later)
- `sensor_msgs` package
- `ros2 bag` command available
- Joystick publishing to `/joy` topic
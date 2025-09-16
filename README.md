# Pupper V3 Codebase

# Install
```sh
sudo apt install git-lfs
git lfs install
git clone https://github.com/Nate711/pupperv3-monorepo.git --recurse-submodules
```

# Docs

Please see the [docs](https://pupper-v3-documentation.readthedocs.io/en/latest/)!

# Notes
* Camera FPS is 1hz by default. Adjustable in `ros2_ws/src/neural_controller/launch/launch.py`

# Development

## Adding animations
#. Hold L1 until BAG status icon turns green to indicate mcap bag recording in process
#. Move Pupper through desired motion
#. Press R1 to stop recording
#. View recorded mcap file in foxglove to verify animation
#. Use `scripts/mcap_to_csv.py [path_to_mcap] -s ABSOLUTE_START_TIME -e ABSOLUTE_END_TIME` to convert to csv
#. Copy csv to ros2_ws/src/animation_controller_py/launch/animations
#. Update pupster.py with animation nickname
#. If editing animation frame rate or fade time, make sure to edit both config.yaml and pupster.yaml

## Camera
```
ros2 launch hailo detection_with_mock_camera_launch.py
```

```
ros2 run foxglove_bridge foxglove_bridge
```
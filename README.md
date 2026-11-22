# ZhuangZi (庄子)

A voice-controlled quadruped built on Stanford's [Pupper V3](https://pupper-v3-documentation.readthedocs.io/en/latest/).
Understands English and Chinese.

## Showcase

**Voice demo**: _add video_

<!-- Embed: drag a video file in on github.com, or
[![ZhuangZi voice demo](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=VIDEO_ID) -->

## Running

```sh
robot/utils/start_zhuangzi.sh [flags]   # robot stack + voice, tmux session "zhuangzi"
robot/utils/stop_zhuangzi.sh            # stops voice, then the robot stack
```

Legs home on start. Voice starts once `controller_manager` is up. Detach: Ctrl+B, D. Reattach:
`tmux attach -t zhuangzi`.

| Flag | Effect | Default |
|---|---|---|
| `--max-distance-m M` | Refuse moves longer than M meters | no limit |
| `--max-turn-deg D` | Refuse turns larger than D degrees | no limit |
| `--fence-cm S` | Refuse moves that could leave an S cm square around the start (dead reckoning, 30% margin) | off |
| `--text` | Type commands instead of speaking (`zh:` prefix for Chinese) | mic |
| `--no-robot` | Log commands without executing | off |

Battery: `python3 robot/utils/check_batt_voltage.py` (0% = 16.5 V, about 1%/min while running).

## Voice commands

Address it as "ZhuangZi". Short commands ("stop", "spin around") work without the name.

| Command | Examples | Behavior |
|---|---|---|
| `stand_up` | "stand up", "start walking", "站起来" | Turns on the walking policy. Required before moving. |
| `move` | "walk forward two meters", "back up a little", "往右走半米" | Walks toward any direction without turning (default 0.5 m). Open loop, 10-30% distance error. |
| `turn` | "turn left", "turn right 45 degrees", "顺时针转一百八十度" | Rotates in place, left = counterclockwise (default clockwise 90°). Closed loop on the gyro. |
| `stop` | "stop", "halt", "停", "别动" | Halts within 50 ms, drops queued commands, stays standing. |

Commands chain in order ("turn left and walk two meters"). A stop word halts the robot about
0.7 s after speech ends, before Haiku is called. Background talk is ignored. Details:
[`ai/listen_and_respond/README.md`](ai/listen_and_respond/README.md).

## Joystick (PS5)

| Input | Action |
|---|---|
| Left stick | Walk forward/back (0.75 m/s), sideways (0.5 m/s) |
| Right stick X | Turn (2 rad/s) |
| X / O | 4-legged / 3-legged walking policy |
| R3 | Emergency stop |
| Options | Resume after e-stop |
| Hold L1 / R1 | Start / stop rosbag recording |

The joystick overrides voice. Pair with `robot/utils/pair_ps5_controller.sh`.

## Architecture

- **Joystick**: `joy_linux` → `teleop_twist_joy` (`/teleop_cmd_vel`); `estop_controller` handles buttons.
- **Voice**: SenseVoice STT → Claude Haiku → `robot.py` (controller switches, `/llm_cmd_vel`).
- **Arbitration**: `cmd_vel_mux` picks joystick > voice → `/cmd_vel`.
- **Control**: `controller_manager` at 520 Hz; `neural_controller` walking policy at ~50 Hz;
  forward controllers for animations; SPI → control board → CAN → 12 motors.

| Path | Contents |
|---|---|
| `ros2_ws/` | ROS 2 workspace; `neural_controller/launch/{launch.py,config.yaml}` configure the robot |
| `ai/listen_and_respond/` | Voice stack |
| `robot/utils/` | Start/stop scripts, battery check, controller pairing |
| `pupper-rs/` | On-screen GUI |
| `ai/rl/` | Policy training |

## Roadmap

- **Speech generation**: text to speech so ZhuangZi speaks, in English and Chinese, with a young
  adult voice. Candidates: Kokoro or MeloTTS on the Pi (sherpa-onnx), or Cartesia / ElevenLabs.
- **Responding to talking**: spoken in-character replies when someone talks to ZhuangZi, with the
  mic muted or echo-cancelled while it speaks.

## Simulation (x86 Ubuntu 24)

```sh
sudo apt install git-lfs && git lfs install
git clone https://github.com/autonomouscrow/ZhuangZi.git --recurse-submodules
./install_dev_dependencies.sh
cd ros2_ws && source build.sh
ros2 launch neural_controller launch.py sim:=True
```

Real robot setup: Pupper V3 [software installation guide](https://pupper-v3-documentation.readthedocs.io/en/latest/guide/software_installation.html).

## Development

### Adding animations

1. Hold L1 until the BAG icon turns green (mcap recording)
1. Move the robot through the motion
1. Press R1 to stop
1. Check the mcap in Foxglove, move it to `bags/`
1. `scripts/mcap_to_csv.py [path_to_mcap] -s ABSOLUTE_START_TIME -e ABSOLUTE_END_TIME`
1. Copy the csv to `ros2_ws/src/animation_controller_py/launch/animations`
1. Rebuild: `./build.sh`
1. Frame rate and fade time: `animation_controller_py` in `ros2_ws/src/neural_controller/launch/config.yaml`

## Credits

Based on [Pupper V3](https://pupper-v3-documentation.readthedocs.io/en/latest/)
([upstream](https://github.com/Nate711/pupperv3-monorepo)). See [LICENSE](LICENSE).

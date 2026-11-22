# listen_and_respond

ZhuangZi's voice stack: hear speech, decide whether it is a physical command,
conversation with ZhuangZi, or background talk, then run the command on the robot or reply.

Stages:
1. **Speech to text** (`stt.py`): Silero VAD + SenseVoice-Small via sherpa-onnx, on the Pi.
   English and Chinese (also Cantonese, Japanese, Korean).
2. **Understanding** (`understand.py`): Claude Haiku 5.5 classifies each utterance as
   `command` / `conversation` / `background` and returns validated JSON. The command
   vocabulary lives in `commands.py` (model-independent); the persona and rules are in
   `system_prompt.md`. Commands are appended to `logs/commands.jsonl`;
   conversation replies are printed (spoken once text to speech exists).
3. **Execution** (`robot.py`): a ROS 2 node that sends each command to the running robot stack.
   Implemented:
   - `stand_up`: turns on the 4-legged walking policy through `/controller_manager/switch_controller`,
     like the joystick's X button.
   - `move`: walks toward any heading without turning (0 = ahead, 90 = right, 180 = back,
     270 = left; default 0.5 m, at most 3 m). Open loop on `/llm_cmd_vel`, so distance is
     approximate (expect 10-30% off).
   - `turn`: rotates in place, clockwise or counterclockwise (default clockwise 90 degrees, at most
     720), closed loop on the IMU's yaw rate. Stops and reports a failure if it turns the wrong
     way, stalls, or loses IMU data.
   - "Turn left then walk 2 m" comes from Haiku as two commands, a `turn` then a `move`.

   - `stop`: interrupts whatever is running within one 50 ms control step, sends zero velocity,
     and drops queued commands. ZhuangZi stays standing. A stop word in the transcript ("stop",
     "halt", "freeze", 停, 别动) halts it before Haiku is even asked, about 0.7 s after you finish
     speaking (0.5 s of silence ends the utterance, then ~0.2 s to decode). It matches any
     "stop", so "don't stop" stops it too. Steps after a stop in the same sentence ("stop, then
     turn left") still run.

   Commands run in order on a background thread (`runner.py`), so ZhuangZi keeps listening while
   it moves. `move` and `turn` need walking on first ("stand up"). Other actions print
   `not implemented yet`. If a step fails, the remaining steps of that utterance are skipped.
4. Text to speech: not decided yet.

## Setup

```bash
cd ai/listen_and_respond
./download_models.sh   # ~240 MB into models/ (gitignored)
uv sync
echo "ANTHROPIC_API_KEY=sk-ant-..." > .env   # gitignored; key from console.anthropic.com
```

Audio uses the system default input (the C920 webcam mic through PulseAudio/PipeWire).

## Run

Start the robot stack first (`robot/utils/robot.sh`), then in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash   # rclpy and the controller_manager messages come from ROS
cd ai/listen_and_respond
uv run listen-and-respond
```

The voice process is its own ROS 2 node, so it can start before or after the robot stack. If the
stack is not up it warns at startup, and each command prints `FAILED` until the stack is running.

Options: `--text` types utterances instead of using the mic (prefix `zh:` for Chinese), to test
classification quickly; `--no-robot` logs commands without sending them (no ROS needed);
`--stt-only` skips understanding (no API key needed).

Prints one line per utterance with the detected language, decode time, and `CLIPPED` if the
mic input hit full scale (clipping lowers recognition accuracy; lower the input volume with
`pactl set-source-volume @DEFAULT_SOURCE@ <percent>%`).

## Notes

- `sherpa-onnx-core` is listed explicitly in `pyproject.toml`: the published metadata for
  sherpa-onnx 1.13.8 omits it, but the aarch64 wheel needs its `libonnxruntime.so`.
  Keep both pinned to the same version.

Example `logs/commands.jsonl` line:

```json
{"time": "2026-10-10T16:02:11+0800", "heard": "ZhuangZi, go forward then spin around.", "lang": "en",
 "commands": [{"action": "move", "heading_deg": 0, "distance_m": null, "rotation": null, "angle_deg": null, "trick": null},
              {"action": "turn", "heading_deg": null, "distance_m": null, "rotation": null, "angle_deg": 360, "trick": null}]}
```

- API errors (no network, rate limits after the SDK's 2 retries) stop the program rather than
  being skipped, so failures are visible.

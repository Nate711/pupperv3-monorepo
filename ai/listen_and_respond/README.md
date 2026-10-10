# listen_and_respond

ZhuangZi's voice stack: hear speech, decide whether it is a physical command,
conversation with ZhuangZi, or background talk, then log the command or reply.

Stages:
1. **Speech to text** (`stt.py`): Silero VAD + SenseVoice-Small via sherpa-onnx, on the Pi.
   English and Chinese (also Cantonese, Japanese, Korean).
2. **Understanding** (`understand.py`): Claude Haiku 5.5 classifies each utterance as
   `command` / `conversation` / `background` and returns validated JSON. The command
   vocabulary lives in `commands.py` (model-independent); the persona and rules are in
   `system_prompt.md`. Commands are appended to `logs/commands.jsonl` (not executed yet);
   conversation replies are printed (spoken once step 3 exists).
3. Text to speech: not decided yet.

## Setup

```bash
cd ai/listen_and_respond
./download_models.sh   # ~240 MB into models/ (gitignored)
uv sync
echo "ANTHROPIC_API_KEY=sk-ant-..." > .env   # gitignored; key from console.anthropic.com
```

Audio uses the system default input (the C920 webcam mic through PulseAudio/PipeWire).

## Run

```bash
uv run listen-and-respond
```

Options: `--text` types utterances instead of using the mic (prefix `zh:` for Chinese), to test
classification quickly; `--stt-only` skips understanding (no API key needed).

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
 "commands": [{"action": "move", "direction": "forward", "distance_m": null, "angle_deg": null, "trick": null},
              {"action": "turn", "direction": null, "distance_m": null, "angle_deg": 360, "trick": null}]}
```

- API errors (no network, rate limits after the SDK's 2 retries) stop the program rather than
  being skipped, so failures are visible.

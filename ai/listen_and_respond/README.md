# listen_and_respond

ZhuangZi's voice stack: hear speech, decide whether it is a physical command,
conversation with ZhuangZi, or background talk, then log the command or reply.

Stages:
1. **Speech to text** (`stt.py`): Silero VAD + SenseVoice-Small via sherpa-onnx, on the Pi.
   English and Chinese (also Cantonese, Japanese, Korean).
2. Understanding: Claude Haiku returning JSON (not built yet).
3. Text to speech: not decided yet.

## Setup

```bash
cd ai/listen_and_respond
./download_models.sh   # ~240 MB into models/ (gitignored)
uv sync
```

Audio uses the system default input (the C920 webcam mic through PulseAudio/PipeWire).

## Run

```bash
uv run listen-and-respond
```

Prints one line per utterance with the detected language, decode time, and `CLIPPED` if the
mic input hit full scale (clipping lowers recognition accuracy; lower the input volume with
`pactl set-source-volume @DEFAULT_SOURCE@ <percent>%`).

## Notes

- `sherpa-onnx-core` is listed explicitly in `pyproject.toml`: the published metadata for
  sherpa-onnx 1.13.8 omits it, but the aarch64 wheel needs its `libonnxruntime.so`.
  Keep both pinned to the same version.

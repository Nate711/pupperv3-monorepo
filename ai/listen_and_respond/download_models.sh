#!/bin/bash -e
# Download the speech-to-text models into ./models (gitignored).
#   - Silero VAD: detects when someone starts and stops speaking
#   - SenseVoice-Small (int8): multilingual ASR (zh, en, ja, ko, yue)

cd "$(dirname "$0")"
mkdir -p models
cd models

RELEASE=https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models
SENSEVOICE=sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17

if [ ! -f silero_vad.onnx ]; then
    curl -fL -o silero_vad.onnx "$RELEASE/silero_vad.onnx"
fi

if [ ! -d "$SENSEVOICE" ]; then
    curl -fL -o "$SENSEVOICE.tar.bz2" "$RELEASE/$SENSEVOICE.tar.bz2"
    tar xjf "$SENSEVOICE.tar.bz2"
    rm "$SENSEVOICE.tar.bz2"
    # Only the int8 model is used; drop the fp32 one to save ~700 MB.
    rm -f "$SENSEVOICE/model.onnx"
fi

ls -la silero_vad.onnx "$SENSEVOICE"

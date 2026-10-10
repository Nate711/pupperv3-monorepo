"""Step 1: speech to text.

Reads the default audio input (the C920 webcam mic via PulseAudio), cuts it into
utterances with Silero VAD, and transcribes each one with SenseVoice-Small.
"""

import queue
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import sherpa_onnx
import sounddevice as sd

SAMPLE_RATE = 16000
MODELS_DIR = Path(__file__).resolve().parents[2] / "models"
SENSEVOICE_DIR = MODELS_DIR / "sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17"
VAD_MODEL = MODELS_DIR / "silero_vad.onnx"

# Silero VAD expects 512-sample windows at 16 kHz.
VAD_WINDOW = 512


@dataclass
class Utterance:
    text: str
    lang: str  # "en", "zh", "yue", "ja", "ko", or "" if unknown
    emotion: str  # e.g. "NEUTRAL", "HAPPY"
    event: str  # e.g. "Speech", "Laughter", "Applause"
    duration_s: float  # length of the speech segment
    decode_s: float  # time SenseVoice took to transcribe it
    peak: float  # max |sample| in the segment; 1.0 means the mic clipped


def _strip_tag(tag: str) -> str:
    """SenseVoice reports tags like '<|en|>'; return 'en'."""
    return tag.removeprefix("<|").removesuffix("|>")


class SpeechToText:
    def __init__(self, num_threads: int = 4, min_silence_s: float = 0.5):
        for path in (VAD_MODEL, SENSEVOICE_DIR / "model.int8.onnx", SENSEVOICE_DIR / "tokens.txt"):
            if not path.exists():
                raise FileNotFoundError(f"{path} is missing. Run ./download_models.sh first.")

        self.recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=str(SENSEVOICE_DIR / "model.int8.onnx"),
            tokens=str(SENSEVOICE_DIR / "tokens.txt"),
            use_itn=True,  # adds punctuation and writes numbers as digits
            num_threads=num_threads,
        )

        vad_config = sherpa_onnx.VadModelConfig()
        vad_config.silero_vad.model = str(VAD_MODEL)
        vad_config.silero_vad.threshold = 0.5
        vad_config.silero_vad.min_silence_duration = min_silence_s  # pause that ends an utterance
        vad_config.silero_vad.min_speech_duration = 0.25
        vad_config.silero_vad.max_speech_duration = 20
        vad_config.silero_vad.window_size = VAD_WINDOW
        vad_config.sample_rate = SAMPLE_RATE
        self.vad = sherpa_onnx.VoiceActivityDetector(vad_config, buffer_size_in_seconds=60)

        self._audio: queue.Queue[np.ndarray] = queue.Queue()

    def transcribe(self, samples: np.ndarray) -> Utterance:
        stream = self.recognizer.create_stream()
        stream.accept_waveform(SAMPLE_RATE, samples)
        start = time.monotonic()
        self.recognizer.decode_stream(stream)
        result = stream.result
        return Utterance(
            text=result.text.strip(),
            lang=_strip_tag(result.lang),
            emotion=_strip_tag(result.emotion),
            event=_strip_tag(result.event),
            duration_s=len(samples) / SAMPLE_RATE,
            decode_s=time.monotonic() - start,
            peak=float(np.max(np.abs(samples))),
        )

    def _on_audio(self, indata, frames, time_info, status):
        if status:
            print(f"[audio] {status}", flush=True)
        self._audio.put(indata[:, 0].copy())

    def listen(self):
        """Yield an Utterance for each stretch of speech heard on the mic. Runs forever."""
        pending = np.zeros(0, dtype=np.float32)
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            blocksize=VAD_WINDOW,
            callback=self._on_audio,
        ):
            while True:
                pending = np.concatenate([pending, self._audio.get()])
                while len(pending) >= VAD_WINDOW:
                    self.vad.accept_waveform(pending[:VAD_WINDOW])
                    pending = pending[VAD_WINDOW:]
                while not self.vad.empty():
                    samples = np.array(self.vad.front.samples, dtype=np.float32)
                    self.vad.pop()
                    utterance = self.transcribe(samples)
                    if utterance.text:
                        yield utterance

# Test file: https://storage.googleapis.com/generativeai-downloads/data/16000.wav
# Install helpers for converting files: pip install librosa soundfile
import asyncio
import io
from pathlib import Path
import wave
from google import genai
from google.genai import types
import soundfile as sf
import librosa

client = genai.Client(api_key="AIzaSyD3Bpij5W1W2HJoBqbvbOhDQKqtkXg-mZk")

# Half cascade model:
# model = "gemini-live-2.5-flash-preview"

# Native audio output model:
model = "gemini-2.5-flash-preview-native-audio-dialog"

config = {
    "response_modalities": ["AUDIO"],
    "system_instruction": "You are a helpful assistant and answer in a friendly tone.",
}


async def main():
    async with client.aio.live.connect(model=model, config=config) as session:

        buffer = io.BytesIO()
        path = "/Users/nathankau/Desktop/test_audio.m4a"
        y, sr = librosa.load(path, sr=16000)
        sf.write(buffer, y, sr, format="RAW", subtype="PCM_16")
        buffer.seek(0)
        audio_bytes = buffer.read()

        # If already in correct format, you can use this:
        # audio_bytes = Path("sample.pcm").read_bytes()

        await session.send_realtime_input(audio=types.Blob(data=audio_bytes, mime_type="audio/pcm;rate=16000"))

        wf = wave.open("audio.wav", "wb")
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(24000)  # Output is 24kHz

        async for response in session.receive():
            # print(response)
            if response.data is not None:
                wf.writeframes(response.data)

            # Un-comment this code to print audio data info
            if response.server_content.model_turn is not None:
                print(response.server_content.model_turn.parts[0].inline_data.mime_type)

        wf.close()


if __name__ == "__main__":
    asyncio.run(main())

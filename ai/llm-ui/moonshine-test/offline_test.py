import moonshine_onnx  # or import moonshine_onnx
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(script_dir, "test_audio.m4a")
print(moonshine_onnx.transcribe(path, "moonshine/tiny"))

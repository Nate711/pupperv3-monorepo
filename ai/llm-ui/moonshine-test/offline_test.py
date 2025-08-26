import moonshine_onnx  # or import moonshine_onnx

# path = "/Users/nathankau/Desktop/test audio pupster.m4a"
path = "/Users/nathankau/Desktop/test audio 2.m4a"
print(moonshine_onnx.transcribe(path, "moonshine/base"))

from fastrtc import Stream, ReplyOnPause
import numpy as np


def echo(audio: tuple[int, np.ndarray]):
    # The function will be passed the audio until the user pauses
    # Implement any iterator that yields audio
    # See "LLM Voice Chat" for a more complete example
    yield audio


stream = Stream(
    handler=ReplyOnPause(echo),
    modality="audio",
    mode="send-receive",
)

stream.ui.launch()

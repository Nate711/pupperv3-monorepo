# example requires websocket-client library:
# pip install websocket-client

import base64
import os
import json
from pathlib import Path
import time
import websocket

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY")

url = "wss://api.openai.com/v1/realtime?model=gpt-realtime"
headers = ["Authorization: Bearer " + OPENAI_API_KEY]


def load_image(image_path):
    with open(image_path, "rb") as f:
        image_base64 = base64.b64encode(f.read()).decode("utf-8")
        return image_base64


def on_open(ws):
    print("Connected to server.")
    start_time = time.time()

    event = {
        "type": "session.update",
        "session": {
            "type": "realtime",
            "model": "gpt-realtime",
            # Lock the output to audio (add "text" if you also want text)
            "output_modalities": ["text"],
            "audio": {
                "input": {
                    "format": {
                        "type": "audio/pcm",
                        "rate": 24000,
                        "channels": 1,
                    },
                    "turn_detection": {"type": "semantic_vad"},
                },
                "output": {
                    "format": {
                        "type": "audio/pcmu",
                    },
                    "voice": "marin",
                },
            },
            # You can still set direct session fields; these override prompt fields if they overlap:
            "instructions": "You are an expert at analyzing spatial relations in images",
        },
    }
    print(f"Sending session.update at {time.time() - start_time:.2f}s")
    send_start = time.time()
    ws.send(json.dumps(event))
    print(f"session.update send took {time.time() - send_start:.4f}s")

    image_dir = (Path(__file__).parent / "images").resolve()

    # walk the image_dir
    for image_path in image_dir.glob("*.jpg"):
        print(f"Sending image {image_path} at {time.time() - start_time:.2f}s")
        img64 = load_image(image_path)
        event = {
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": "Describe the location of the person in the image in two numbers. x: 0 to 1 (left to right) and y: 0 to 1 (top to bottom) in 2 decimals. Example output: '0.45, 0.75'",
                    },
                    {"type": "input_image", "image_url": f"data:image/jpeg;base64,{img64}"},
                ],
            },
        }
        send_start = time.time()
        ws.send(json.dumps(event))
        print(f"conversation.item.create send took {time.time() - send_start:.4f}s")

        event = {"type": "response.create", "response": {"output_modalities": ["text"]}}
        send_start = time.time()
        ws.send(json.dumps(event))
        print(f"response.create send took {time.time() - send_start:.4f}s")
        break


def on_message(ws, message):
    data = json.loads(message)
    if data.get("type") == "response.output_text.delta":
        return
    # print("Received event:", json.dumps(data, indent=2))
    print(f"Received event at {time.time():.2f}. Type:", data.get("type"))
    if data.get("type") == "error":
        print("Error from server:", data.get("message"))
    if data.get("type") == "response.done":
        print("Final response", json.dumps(data, indent=2))
        print("\n\n\n")


ws = websocket.WebSocketApp(
    url,
    header=headers,
    on_open=on_open,
    on_message=on_message,
)

ws.run_forever()

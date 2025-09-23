"""WebSocket-based benchmarking for OpenAI Realtime API."""

import asyncio
import base64
import json
import time
from pathlib import Path
from typing import Dict
import websockets


class RealtimeBenchmark:
    """Benchmark OpenAI Realtime API using WebSockets."""

    def __init__(self, api_key: str):
        """Initialize with API key."""
        self.api_key = api_key
        self.ws_url = "wss://api.openai.com/v1/realtime?model=gpt-realtime"
        self.model = "gpt-realtime"

    async def benchmark_image(self, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single image using the Realtime API.

        Note: As of 2025, the Realtime API primarily supports audio/voice.
        Image support is limited or may not be available.
        """
        # Encode image to base64
        with open(image_path, "rb") as f:
            image_base64 = base64.b64encode(f.read()).decode("utf-8")

        start_time = time.perf_counter()

        try:
            # Connect to WebSocket using official format
            headers = [f"Authorization: Bearer {self.api_key}"]

            async with websockets.connect(self.ws_url, additional_headers=headers) as websocket:
                # Send session update to configure the model
                session_update = {
                    "type": "session.update",
                    "session": {
                        "type": "realtime",
                        "instructions": "You are a helpful assistant that describes images in detail.",
                        "modalities": ["text"],
                    },
                }
                await websocket.send(json.dumps(session_update))

                # Wait for initial messages (session.created, etc.)
                initial_messages = []
                for _ in range(3):  # Wait for a few initial messages
                    try:
                        response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                        message = json.loads(response)
                        initial_messages.append(message)
                        if message.get("type") == "session.updated":
                            break
                    except asyncio.TimeoutError:
                        break

                # Note: Realtime API is primarily for audio, but trying text with image description
                # Create a text message describing that we want to analyze an image
                text_prompt = f"{prompt}\n\nImage data (base64): {image_base64[:100]}..."  # Truncated for demo

                conversation_item = {
                    "type": "conversation.item.create",
                    "item": {
                        "type": "message",
                        "role": "user",
                        "content": [{"type": "input_text", "text": text_prompt}],
                    },
                }
                await websocket.send(json.dumps(conversation_item))

                # Request a response
                response_create = {"type": "response.create", "response": {"modalities": ["text"]}}
                await websocket.send(json.dumps(response_create))

                # Collect response messages
                response_text = ""
                timeout_count = 0
                max_timeouts = 10

                while timeout_count < max_timeouts:
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        data = json.loads(message)

                        # Handle different response types
                        if data["type"] == "response.text.delta":
                            response_text += data.get("delta", "")
                        elif data["type"] == "response.text.done":
                            response_text += data.get("text", "")
                        elif data["type"] == "response.done":
                            break
                        elif data["type"] == "error":
                            raise ValueError(f"API Error: {data.get('error')}")
                        elif data["type"] in [
                            "session.created",
                            "session.updated",
                            "conversation.item.created",
                            "response.created",
                        ]:
                            # Expected session management messages
                            continue

                        # Reset timeout counter on successful message
                        timeout_count = 0

                    except asyncio.TimeoutError:
                        timeout_count += 1
                        if response_text:  # Got some response, might be done
                            break
                        continue

                end_time = time.perf_counter()
                latency = end_time - start_time

                return {
                    "model": self.model,
                    "image": image_path.name,
                    "latency": latency,
                    "response": response_text,
                    "tokens_used": None,  # Realtime API may not provide token counts
                    "success": True,
                    "error": None,
                }

        except Exception as e:
            end_time = time.perf_counter()
            latency = end_time - start_time

            return {
                "model": self.model,
                "image": image_path.name,
                "latency": latency,
                "response": None,
                "tokens_used": None,
                "success": False,
                "error": str(e),
            }

    async def run_benchmarks(self, image_paths: list[Path], prompt: str, delay: float = 0.5):
        """Run benchmarks for multiple images."""
        results = []

        for i, image_path in enumerate(image_paths, 1):
            print(f"[{i}/{len(image_paths)}] Testing {self.model} on {image_path.name}...")

            result = await self.benchmark_image(image_path, prompt)

            if result["success"]:
                print(f"  ✓ Completed in {result['latency']:.2f}s")
            else:
                print(f"  ✗ Failed: {result['error']}")

            results.append(result)

            if i < len(image_paths):
                await asyncio.sleep(delay)

        return results

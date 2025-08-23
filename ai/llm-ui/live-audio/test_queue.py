#!/usr/bin/env python3
"""
Test script for the command queue functionality.
This will send a sequence of commands to demonstrate the queue system.
"""

import asyncio
import json
import websockets


async def test_command_queue():
    """Test the command queue by sending multiple commands."""
    try:
        # Connect to the WebSocket server
        uri = "ws://localhost:8008"
        print(f"Connecting to {uri}...")

        async with websockets.connect(uri) as websocket:
            print("Connected! Sending test command sequence...")

            # Test sequence: activate, wait 2 seconds, move forward, wait 1 second, stop, deactivate
            commands = [
                {"name": "activate", "args": {}, "request_id": "test_1"},
                {"name": "wait", "args": {"duration": 2.0}, "request_id": "test_2"},
                {"name": "move", "args": {"vx": 0.5, "vy": 0, "wz": 0}, "request_id": "test_3"},
                {"name": "wait", "args": {"duration": 1.0}, "request_id": "test_4"},
                {"name": "move", "args": {"vx": 0, "vy": 0, "wz": 0}, "request_id": "test_5"},
                {"name": "deactivate", "args": {}, "request_id": "test_6"},
            ]

            # Send all commands rapidly to queue them
            for cmd in commands:
                print(f"Sending: {cmd['name']} (request_id: {cmd['request_id']})")
                await websocket.send(json.dumps(cmd))

                # Wait briefly to receive the queued acknowledgment
                response = await websocket.recv()
                response_data = json.loads(response)
                print(f"  Queue acknowledgment: {response_data.get('status')} - {response_data.get('message')}")

            # Now wait for all command execution responses
            print("\nWaiting for command execution responses...")
            for i in range(len(commands)):
                response = await websocket.recv()
                response_data = json.loads(response)
                print(
                    f"Response {i+1}: {response_data.get('request_id')} - {response_data.get('status')} - {response_data.get('message')}"
                )

            print("\nTest complete!")

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    asyncio.run(test_command_queue())

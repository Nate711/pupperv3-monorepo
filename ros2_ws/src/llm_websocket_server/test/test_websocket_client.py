#!/usr/bin/env python3
"""
Test client for the LLM WebSocket server.
This script can be used to test the server functionality without the web app.
"""

import asyncio
import json
import websockets
import sys

async def test_commands():
    """Test various robot commands via WebSocket."""
    uri = "ws://localhost:8765"
    
    test_commands = [
        {"name": "status", "args": {}},
        {"name": "get_battery", "args": {}},
        {"name": "activate", "args": {}},
        {"name": "move", "args": {"vx": 0.3, "vy": 0.0, "wz": 0.0}},
        {"name": "move", "args": {"vx": 0.0, "vy": 0.0, "wz": 0.5}},
        {"name": "move", "args": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
        {"name": "deactivate", "args": {}},
        {"name": "status", "args": {}},
    ]
    
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to {uri}")
            
            for i, command in enumerate(test_commands):
                print(f"\n--- Test {i+1}: {command['name']} ---")
                print(f"Sending: {json.dumps(command, indent=2)}")
                
                # Send command
                await websocket.send(json.dumps(command))
                
                # Receive response
                response = await websocket.recv()
                response_data = json.loads(response)
                print(f"Response: {json.dumps(response_data, indent=2)}")
                
                # Wait a bit between commands
                await asyncio.sleep(1)
                
    except (ConnectionRefusedError, OSError):
        print("❌ Could not connect to WebSocket server at ws://localhost:8765")
        print("Make sure the server is running with: ros2 run llm_websocket_server websocket_server")
    except Exception as e:
        print(f"❌ Error: {e}")

async def interactive_mode():
    """Interactive mode for manual testing."""
    uri = "ws://localhost:8765"
    
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to {uri}")
            print("Interactive mode - type commands (or 'quit' to exit):")
            print("Examples:")
            print("  activate")
            print("  move 0.3 0 0")
            print("  get_battery")
            print("  deactivate")
            
            while True:
                user_input = input("\n> ").strip()
                
                if user_input.lower() == 'quit':
                    break
                
                # Parse simple commands
                parts = user_input.split()
                if not parts:
                    continue
                    
                command_name = parts[0]
                
                if command_name == "move" and len(parts) == 4:
                    try:
                        vx, vy, wz = map(float, parts[1:4])
                        command = {"name": "move", "args": {"vx": vx, "vy": vy, "wz": wz}}
                    except ValueError:
                        print("❌ Invalid move command. Use: move <vx> <vy> <wz>")
                        continue
                else:
                    command = {"name": command_name, "args": {}}
                
                print(f"Sending: {json.dumps(command)}")
                
                try:
                    await websocket.send(json.dumps(command))
                    response = await websocket.recv()
                    response_data = json.loads(response)
                    print(f"Response: {response_data.get('message', 'No message')}")
                    
                    if response_data.get('status') == 'error':
                        print(f"❌ Error: {response_data.get('message')}")
                        
                except Exception as e:
                    print(f"❌ Error sending command: {e}")
                    
    except (ConnectionRefusedError, OSError):
        print("❌ Could not connect to WebSocket server at ws://localhost:8765")
        print("Make sure the server is running with: ros2 run llm_websocket_server websocket_server")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    """Main entry point."""
    if len(sys.argv) > 1 and sys.argv[1] == "interactive":
        asyncio.run(interactive_mode())
    else:
        asyncio.run(test_commands())

if __name__ == "__main__":
    main()
# example requires websocket-client library:
# pip install websocket-client

import base64
import os
import json
from pathlib import Path
import time
import websocket
import yaml
import re
from PIL import Image, ImageDraw, ImageFont

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY")

url = "wss://api.openai.com/v1/realtime?model=gpt-realtime"
headers = ["Authorization: Bearer " + OPENAI_API_KEY]

# Global state tracking
current_image_start_time = None
current_image_name = None
image_queue = []
current_image_index = 0
ws_connection = None
ground_truth = {}
results = []  # Store results for summary

# Image dimensions (from sips command)
IMG_WIDTH = 1400
IMG_HEIGHT = 1050


def load_image(image_path):
    with open(image_path, "rb") as f:
        image_base64 = base64.b64encode(f.read()).decode("utf-8")
        return image_base64


def create_annotated_images():
    """Create annotated images showing LLM estimates vs ground truth"""
    from collections import defaultdict

    # Group results by image name
    image_stats = defaultdict(list)
    for result in results:
        if 'error_x_pixels' in result:  # Only valid results
            image_stats[result["image"]].append(result)

    image_dir = Path(__file__).parent / "images"

    for image_name, runs in image_stats.items():
        if not runs:
            continue

        # Load the original image
        img_path = image_dir / image_name
        img = Image.open(img_path)
        draw = ImageDraw.Draw(img)

        # Get ground truth for this image
        gt = ground_truth.get(image_name, {})
        if gt:
            gt_x_px = int(gt['x'] * IMG_WIDTH)
            gt_y_px = int(gt['y'] * IMG_HEIGHT)

            # Draw ground truth as a green cross
            cross_size = 20
            draw.line([(gt_x_px - cross_size, gt_y_px), (gt_x_px + cross_size, gt_y_px)], fill="green", width=3)
            draw.line([(gt_x_px, gt_y_px - cross_size), (gt_x_px, gt_y_px + cross_size)], fill="green", width=3)

            # Draw a circle around the ground truth
            radius = 15
            draw.ellipse([gt_x_px - radius, gt_y_px - radius, gt_x_px + radius, gt_y_px + radius],
                        outline="green", width=2)

        # Draw LLM estimates for each run
        colors = ["red", "blue", "yellow", "magenta", "cyan"]
        for i, run in enumerate(runs[:5]):  # Max 5 runs
            llm_x_px = int(run['llm_x'] * IMG_WIDTH)
            llm_y_px = int(run['llm_y'] * IMG_HEIGHT)
            color = colors[i % len(colors)]

            # Draw LLM estimate as a colored X
            x_size = 15
            draw.line([(llm_x_px - x_size, llm_y_px - x_size), (llm_x_px + x_size, llm_y_px + x_size)],
                     fill=color, width=2)
            draw.line([(llm_x_px - x_size, llm_y_px + x_size), (llm_x_px + x_size, llm_y_px - x_size)],
                     fill=color, width=2)

            # Draw run number next to the X
            try:
                # Try to get a font, fallback to default if not available
                font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 20)
            except:
                font = ImageFont.load_default()

            draw.text((llm_x_px + x_size + 5, llm_y_px - 10), f"{i+1}", fill=color, font=font)

        # Add legend
        legend_y = 30
        legend_x = 30

        # Draw legend background
        legend_height = 30 + len(runs[:5]) * 25 + 30
        draw.rectangle([legend_x - 10, legend_y - 10, legend_x + 250, legend_y + legend_height],
                      fill="white", outline="black", width=2)

        # Legend items
        draw.text((legend_x, legend_y), "Ground Truth (Green +)", fill="green", font=font)
        legend_y += 30

        for i, run in enumerate(runs[:5]):
            color = colors[i % len(colors)]
            error_px = run['euclidean_pixels']
            draw.text((legend_x, legend_y), f"Run {i+1} ({color}) - Error: {error_px:.1f}px", fill=color, font=font)
            legend_y += 25

        # Save the annotated image
        output_path = image_dir / f"annotated_{image_name}"
        img.save(output_path)
        print(f"Saved annotated image: {output_path.name}")


def print_summary_table():
    """Print a summary table of results grouped by image"""
    from collections import defaultdict

    # Create annotated images first
    create_annotated_images()

    # Group results by image name
    image_stats = defaultdict(list)
    for result in results:
        image_stats[result["image"]].append(result)

    print("\n" + "=" * 130)
    print("SUMMARY TABLE - LLM Position Estimates vs Ground Truth")
    print("=" * 130)

    # Header
    print(
        f"{'Image Name':<50} {'Run':<5} {'GT (x,y)':<15} {'LLM (x,y)':<15} {'Error X (px)':<12} {'Error Y (px)':<12} {'Euclidean (px)':<15} {'Time (s)':<10}"
    )
    print("-" * 130)

    # Print results for each image
    total_error_x = 0
    total_error_y = 0
    total_euclidean = 0
    total_time = 0
    total_runs = 0

    for image_name in sorted(image_stats.keys()):
        runs = image_stats[image_name]
        for i, run in enumerate(runs, 1):
            if "error_x_pixels" in run:  # Only include runs with valid results
                gt_str = f"({run['gt_x']:.3f},{run['gt_y']:.3f})"
                llm_str = f"({run['llm_x']:.3f},{run['llm_y']:.3f})"
                print(
                    f"{image_name:<50} {i:<5} {gt_str:<15} {llm_str:<15} {run['error_x_pixels']:>11.1f} {run['error_y_pixels']:>12.1f} {run['euclidean_pixels']:>14.1f} {run['time']:>9.2f}"
                )
                total_error_x += run["error_x_pixels"]
                total_error_y += run["error_y_pixels"]
                total_euclidean += run["euclidean_pixels"]
                total_time += run["time"]
                total_runs += 1

        # Print average for this image if multiple runs
        if len(runs) > 1:
            valid_runs = [r for r in runs if "error_x_pixels" in r]
            if valid_runs:
                avg_x = sum(r["error_x_pixels"] for r in valid_runs) / len(valid_runs)
                avg_y = sum(r["error_y_pixels"] for r in valid_runs) / len(valid_runs)
                avg_euclidean = sum(r["euclidean_pixels"] for r in valid_runs) / len(valid_runs)
                avg_time = sum(r["time"] for r in valid_runs) / len(valid_runs)
                # GT is the same for all runs of the same image
                gt_str = f"({valid_runs[0]['gt_x']:.3f},{valid_runs[0]['gt_y']:.3f})"
                print(
                    f"  {'Average:':<48} {'':<5} {gt_str:<15} {'':<15} {avg_x:>11.1f} {avg_y:>12.1f} {avg_euclidean:>14.1f} {avg_time:>9.2f}"
                )
                print("-" * 130)

    # Print overall average
    if total_runs > 0:
        print(
            f"{'OVERALL AVERAGE':<50} {'':<5} {'':<15} {'':<15} {total_error_x/total_runs:>11.1f} {total_error_y/total_runs:>12.1f} {total_euclidean/total_runs:>14.1f} {total_time/total_runs:>9.2f}"
        )
        print("=" * 130)
        print(f"Total runs: {total_runs}")
        print(f"Average processing time: {total_time/total_runs:.2f}s")
        print(f"Average position error: {total_euclidean/total_runs:.1f} pixels")


def send_next_image():
    global current_image_start_time, current_image_name, current_image_index, ws_connection

    if current_image_index >= len(image_queue):
        print(f"\n✓ All {len(image_queue)} images processed")
        print_summary_table()
        return

    image_path = image_queue[current_image_index]
    current_image_name = image_path.name

    print(f"\n[{current_image_index + 1}/{len(image_queue)}] Processing {current_image_name}")

    img64 = load_image(image_path)
    event = {
        "type": "conversation.item.create",
        "item": {
            "type": "message",
            "role": "user",
            "content": [
                {
                    "type": "input_text",
                    "text": "Locate the head of the person in the image in three coordinates. x: 0 to 1 (left to right) and y: 0 to 1 (top to bottom), depth (approx meters) in 2 decimals. Example output: '(0.45, 0.75). depth=1.23'",
                },
                {"type": "input_image", "image_url": f"data:image/jpeg;base64,{img64}"},
            ],
        },
    }

    send_start = time.time()
    ws_connection.send(json.dumps(event))
    print(f"  conversation.item.create send took {time.time() - send_start:.4f}s")

    event = {"type": "response.create", "response": {"output_modalities": ["text"]}}
    send_start = time.time()
    current_image_start_time = time.time()  # Start timing for response
    ws_connection.send(json.dumps(event))
    print(f"  response.create send took {time.time() - send_start:.4f}s")

    current_image_index += 1


def load_ground_truth():
    global ground_truth
    gt_path = Path(__file__).parent / "images" / "gt.yaml"
    with open(gt_path, "r") as f:
        gt_data = yaml.safe_load(f)

    # Convert pixel coordinates to normalized coordinates
    for filename, coords in gt_data.items():
        x_norm = coords["x_pixels"] / IMG_WIDTH
        y_norm = coords["y_pixels"] / IMG_HEIGHT
        ground_truth[filename] = {"x": x_norm, "y": y_norm}

    print(f"Loaded ground truth for {len(ground_truth)} images")


def on_open(ws):
    global ws_connection, image_queue
    ws_connection = ws

    print("Connected to server.")

    # Load ground truth
    load_ground_truth()

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

    # Load all image paths into the queue and repeat them
    image_dir = (Path(__file__).parent / "images").resolve()
    images = sorted(list(image_dir.glob("*.jpg")))
    image_queue = images * 5  # Repeat the list 5 times
    print(f"Found {len(images)} unique images, will process each 5 times ({len(image_queue)} total)")

    # Send the first image
    send_next_image()


def on_message(ws, message):
    global current_image_start_time, current_image_name

    data = json.loads(message)
    event_type = data.get("type")

    # Skip text delta events to reduce noise
    if event_type == "response.output_text.delta":
        return

    # Log other events
    if event_type not in ["response.output_text.done", "response.done"]:
        print(f"  Event: {event_type}")

    if event_type == "error":
        print(f"  ❌ Error: {data.get('message')}")

    if event_type == "response.output_text.done":
        # Extract the actual text response
        text = data.get("item", {}).get("content", [{}])[0].get("text", "")
        if text:
            print(f"  Response: {text}")

    if event_type == "response.done":
        # Initialize result dictionary for this run
        result = {"image": current_image_name, "time": 0}

        # Extract text from the response
        response = data.get("response", {})
        outputs = response.get("output", [])
        llm_text = ""
        if outputs:
            content = outputs[0].get("content", [])
            if content:
                llm_text = content[0].get("text", "")
                if llm_text:
                    print(f"  LLM response: {llm_text}")

                    # Parse the LLM response to extract x, y coordinates
                    # Expected format: "(0.45, 0.75). depth=1.23" or similar
                    coords_match = re.search(r"[(\[]?\s*([0-9.]+)\s*,\s*([0-9.]+)\s*[)\]]?", llm_text)
                    depth_match = re.search(r"depth\s*=\s*([0-9.]+)", llm_text, re.IGNORECASE)

                    if coords_match and current_image_name in ground_truth:
                        llm_x = float(coords_match.group(1))
                        llm_y = float(coords_match.group(2))
                        depth = float(depth_match.group(1)) if depth_match else None

                        gt = ground_truth[current_image_name]
                        error_x = abs(llm_x - gt["x"])
                        error_y = abs(llm_y - gt["y"])
                        error_pixels_x = error_x * IMG_WIDTH
                        error_pixels_y = error_y * IMG_HEIGHT
                        euclidean_error = ((error_x**2) + (error_y**2)) ** 0.5
                        euclidean_error_pixels = ((error_pixels_x**2) + (error_pixels_y**2)) ** 0.5

                        print(f"  Ground truth: ({gt['x']:.3f}, {gt['y']:.3f})")
                        print(f"  LLM estimate: ({llm_x:.3f}, {llm_y:.3f})")
                        if depth:
                            print(f"  Depth estimate: {depth:.2f}m")
                        print(
                            f"  Error: x={error_x:.3f} ({error_pixels_x:.1f}px), y={error_y:.3f} ({error_pixels_y:.1f}px)"
                        )
                        print(f"  Euclidean error: {euclidean_error:.3f} ({euclidean_error_pixels:.1f}px)")

                        # Store results for summary
                        result.update(
                            {
                                "llm_x": llm_x,
                                "llm_y": llm_y,
                                "gt_x": gt["x"],
                                "gt_y": gt["y"],
                                "error_x": error_x,
                                "error_y": error_y,
                                "error_x_pixels": error_pixels_x,
                                "error_y_pixels": error_pixels_y,
                                "euclidean": euclidean_error,
                                "euclidean_pixels": euclidean_error_pixels,
                                "depth": depth,
                            }
                        )

        # Calculate and display response time
        if current_image_start_time and current_image_name:
            response_time = time.time() - current_image_start_time
            result["time"] = response_time
            print(f"  ✓ {current_image_name} processed in {response_time:.2f}s")

        # Store the result
        results.append(result)

        # Send the next image
        send_next_image()


ws = websocket.WebSocketApp(
    url,
    header=headers,
    on_open=on_open,
    on_message=on_message,
)

ws.run_forever()

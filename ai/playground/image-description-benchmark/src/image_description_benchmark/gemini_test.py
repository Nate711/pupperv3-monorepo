from google import genai
from google.genai import types
from PIL import Image
from pathlib import Path

import os
from dotenv import load_dotenv

# Import our visualization functions and types
from image_description_benchmark.bbox_types import BoundingBox, transform_to_pixels
from image_description_benchmark.visualization import parse_bounding_boxes, draw_bounding_boxes

load_dotenv(".env.local", override=True)
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
print(f"API Key loaded: {'Yes' if GOOGLE_API_KEY else 'No'}")

# prompt = "Output a set of 10 points in JSON ([y, x] normalized between 0 to 1000) to reach the kitchen or explore towards it. Make the label the waypoint number."
prompt = "From this point of view as a mobile robot, point out a reachable position on the ground where you should go to try to find the bathroom. "

# prompt = """
# Detect objects that would be obstacles to a vacuum robot.
# Output bounding boxes in JSON format [y1, x1, y2, x2] with a box_2d and label field.
# DO NOT OUTPUT POINTS FOR OBSTACLES. ONLY OUTPUT box_2d bounding boxes.
# Then output a trajectory of 10 points for the vacuum robot that guide it from the door to the kitchen without collision.
# Label the points with waypoint number.
# """

SYSTEM_INSTRUCTIONS = """
Return bounding boxes or points as a JSON array with labels. Never return masks or code fencing. Limit to 25 objects.
If an object is present multiple times, name them according to their unique characteristic (colors, size, position, unique characteristics, etc..).
"""

image_path = Path(
    #     "/Users/nathankau/pupperv3-monorepo/ai/playground/image-description-benchmark/src/image_description_benchmark/images/camera_image_raw_compressed-1755118546-420444431.jpg"
    # "/Users/nathankau/pupperv3-monorepo/untracked_bags/tracking_me_rosbag2_2025_08_13-13_55_08/extracted_images/image_01755118597171220979.jpg"
    # "/Users/nathankau/pupperv3-monorepo/ai/playground/image-description-benchmark/src/image_description_benchmark/images/609abd4bfd29a369ec80dd82_RoomSketcher-Kitchen-Layout-Ideas-3D-Floor-Plan.jpeg"
    "/Users/nathankau/pupperv3-monorepo/ai/playground/image-description-benchmark/src/image_description_benchmark/images/IMG_5907 Medium.jpeg"
)

# Load image
im = Image.open(image_path)

# model_name = "gemini-robotics-er-1.5-preview"
model_name = "gemini-2.5-flash"

client = genai.Client(api_key=GOOGLE_API_KEY)

# Run model to find bounding boxes
response = client.models.generate_content(
    model=model_name,
    contents=[im, prompt],
    config=types.GenerateContentConfig(
        temperature=0.5,
        system_instruction=SYSTEM_INSTRUCTIONS,
        # thinking_config=types.ThinkingConfig(thinking_budget=1024),
    ),
)

# Check output
print("Raw response:")
print(response.text)
print("\n" + "=" * 50)

# Parse bounding boxes from response
# Check config to determine coordinate format
# For now, assuming xyxy format
boxes = parse_bounding_boxes(response.text)
print(f"\nParsed {len(boxes)} bounding boxes:")
# for i, bbox in enumerate(boxes):
#     print(f"  Box {i+1}: {bbox}")

# Draw bounding boxes on image
if boxes:
    output_path = Path("gemini_test_output.jpg")
    annotated_image = draw_bounding_boxes(
        image_path, boxes, output_path, color=(255, 0, 0), width=3, input_coordinate_range=1000  # Red boxes
    )
    print(f"\nSaved annotated image to: {output_path}")
    print("Image dimensions:", im.size)

    # Print coordinate transformation info
    for i, bbox in enumerate(boxes):
        pixel_bbox = transform_to_pixels(bbox, im.size[0], im.size[1], 1000)
        # print(f"Box {i+1} transformed: {bbox} -> {pixel_bbox}")
else:
    print("\nNo bounding boxes found to draw.")

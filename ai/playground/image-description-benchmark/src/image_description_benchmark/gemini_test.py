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

prompt = "Detect persons with no more than 20 items. Output a json list where each entry contains the 2D bounding box in 'box_2d' and a text label in 'label'. "
# prompt = "Describe the image"
image_path = Path(
    "/Users/nathankau/pupperv3-monorepo/ai/playground/image-description-benchmark/src/image_description_benchmark/images/camera_image_raw_compressed-1755118546-420444431.jpg"
)

# Load image
im = Image.open(image_path)

model_name = "gemini-2.5-flash"

client = genai.Client(api_key=GOOGLE_API_KEY)

# Run model to find bounding boxes
response = client.models.generate_content(
    model=model_name,
    contents=[im, prompt],
    config=types.GenerateContentConfig(
        temperature=0.5,
        thinking_config=types.ThinkingConfig(thinking_budget=0),
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
for i, bbox in enumerate(boxes):
    print(f"  Box {i+1}: {bbox}")

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
        print(f"Box {i+1} transformed: {bbox} -> {pixel_bbox}")
else:
    print("\nNo bounding boxes found to draw.")

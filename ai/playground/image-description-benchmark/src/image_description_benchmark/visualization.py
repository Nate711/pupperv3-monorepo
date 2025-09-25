"""Visualization module for plotting bounding boxes on images."""

import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def transform_coordinates(
    box_coords: List[int], image_width: int, image_height: int, input_range: int = 1000
) -> List[int]:
    """
    Transform normalized coordinates to image pixel coordinates.

    Args:
        box_coords: List of 4 coordinates [x1, y1, x2, y2] in normalized range
        image_width: Actual image width in pixels
        image_height: Actual image height in pixels
        input_range: The normalization range used by the model (default: 1000)

    Returns:
        List of 4 coordinates in image pixel space
    """
    if len(box_coords) != 4:
        return box_coords

    x1, y1, x2, y2 = box_coords

    # Transform from normalized coordinates to pixel coordinates
    pixel_x1 = int((x1 / input_range) * image_width)
    pixel_y1 = int((y1 / input_range) * image_height)
    pixel_x2 = int((x2 / input_range) * image_width)
    pixel_y2 = int((y2 / input_range) * image_height)

    # Ensure coordinates are within image bounds
    pixel_x1 = max(0, min(pixel_x1, image_width))
    pixel_y1 = max(0, min(pixel_y1, image_height))
    pixel_x2 = max(0, min(pixel_x2, image_width))
    pixel_y2 = max(0, min(pixel_y2, image_height))

    return [pixel_x1, pixel_y1, pixel_x2, pixel_y2]


def parse_bounding_boxes(response_text: str) -> List[Dict]:
    """
    Parse bounding boxes from model response text.

    Args:
        response_text: Model response containing JSON with bounding boxes

    Returns:
        List of dictionaries with box_2d coordinates and labels
    """
    if not response_text:
        return []

    # Clean up the response text
    cleaned_text = response_text.strip()

    # Remove markdown code blocks if present
    if "```json" in cleaned_text:
        cleaned_text = re.sub(r"```json\s*", "", cleaned_text)
        cleaned_text = re.sub(r"\s*```", "", cleaned_text)

    # Look for JSON array pattern
    json_match = re.search(r"\[\s*\{.*?\}\s*\]", cleaned_text, re.DOTALL)
    if json_match:
        json_str = json_match.group(0)
        boxes = json.loads(json_str)

        # Validate structure
        validated_boxes = []
        for box in boxes:
            if isinstance(box, dict) and "box_2d" in box and "label" in box:
                if isinstance(box["box_2d"], list) and len(box["box_2d"]) == 4:
                    validated_boxes.append(box)

        return validated_boxes

    return []


def draw_bounding_boxes(
    image_path: Path,
    boxes: List[Dict],
    output_path: Optional[Path] = None,
    color: Tuple[int, int, int] = (255, 0, 0),
    width: int = 3,
    font_size: int = 20,
    input_coordinate_range: int = 1000,
) -> Image.Image:
    """
    Draw bounding boxes on an image.

    Args:
        image_path: Path to input image
        boxes: List of bounding box dictionaries with 'box_2d' and 'label' keys
        output_path: Optional path to save the annotated image
        color: RGB color for boxes (default: red)
        width: Line width for boxes
        font_size: Font size for labels
        input_coordinate_range: Range of input coordinates (default: 1000 for normalized coords)

    Returns:
        PIL Image with bounding boxes drawn
    """
    # Load image
    img = Image.open(image_path)
    draw = ImageDraw.Draw(img)
    img_width, img_height = img.size

    # Try to load a font, fallback to default if not available
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", font_size)
    except:
        font = ImageFont.load_default()

    # Draw each bounding box
    for box_info in boxes:
        if "box_2d" not in box_info or "label" not in box_info:
            continue

        coords = box_info["box_2d"]
        label = box_info["label"]

        # Ensure coordinates are in correct format [x1, y1, x2, y2]
        if len(coords) == 4:
            # Transform normalized coordinates to pixel coordinates
            pixel_coords = transform_coordinates(coords, img_width, img_height, input_coordinate_range)
            x1, y1, x2, y2 = pixel_coords

            # Draw rectangle
            draw.rectangle([x1, y1, x2, y2], outline=color, width=width)

            # Draw label background - make sure label fits within image
            label_y = max(y1 - 25, 0)  # Don't go above image
            try:
                label_bbox = draw.textbbox((x1, label_y), label, font=font)
                draw.rectangle(label_bbox, fill=color)
                # Draw label text
                draw.text((x1, label_y), label, fill=(255, 255, 255), font=font)
            except:
                # Fallback if textbbox is not available (older PIL versions)
                draw.text((x1, label_y), label, fill=color, font=font)

    # Save if output path provided
    if output_path:
        img.save(output_path)

    return img


def create_comparison_grid(
    image_path: Path,
    results: List[Dict],
    output_path: Path,
    max_cols: int = 3,
) -> Image.Image:
    """
    Create a grid showing original image and model predictions side by side.

    Args:
        image_path: Path to original image
        results: List of model results with 'model', 'response', and 'success' keys
        output_path: Path to save the comparison grid
        max_cols: Maximum columns in the grid

    Returns:
        PIL Image containing the comparison grid
    """
    # Filter successful results
    successful_results = [r for r in results if r.get("success", False)]

    if not successful_results:
        print("No successful results to visualize")
        return None

    # Load original image
    original_img = Image.open(image_path)
    img_width, img_height = original_img.size

    # Calculate grid dimensions
    n_images = len(successful_results) + 1  # +1 for original
    n_cols = min(max_cols, n_images)
    n_rows = (n_images + n_cols - 1) // n_cols

    # Create grid canvas
    padding = 20
    label_height = 40
    grid_width = n_cols * img_width + (n_cols + 1) * padding
    grid_height = n_rows * (img_height + label_height) + (n_rows + 1) * padding

    grid_img = Image.new("RGB", (grid_width, grid_height), color=(240, 240, 240))
    draw = ImageDraw.Draw(grid_img)

    # Try to load font
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 24)
    except:
        font = ImageFont.load_default()

    # Paste original image
    x_offset = padding
    y_offset = padding
    grid_img.paste(original_img, (x_offset, y_offset + label_height))

    # Draw label for original
    label = "Original"
    draw.text((x_offset, y_offset), label, fill=(0, 0, 0), font=font)

    # Process each model result
    position = 1
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 128, 0), (128, 0, 255)]

    for idx, result in enumerate(successful_results):
        # Parse bounding boxes from response
        boxes = parse_bounding_boxes(result.get("response", ""))

        # Create annotated image
        color = colors[idx % len(colors)]
        annotated_img = draw_bounding_boxes(image_path, boxes, color=color, width=3, input_coordinate_range=1000)

        # Calculate position in grid
        row = position // n_cols
        col = position % n_cols
        x_offset = col * img_width + (col + 1) * padding
        y_offset = row * (img_height + label_height) + (row + 1) * padding

        # Paste annotated image
        grid_img.paste(annotated_img, (x_offset, y_offset + label_height))

        # Draw model label
        model_name = result.get("model", "Unknown")
        n_boxes = len(boxes)
        label = f"{model_name} ({n_boxes} boxes)"
        draw.text((x_offset, y_offset), label, fill=(0, 0, 0), font=font)

        position += 1

    # Save grid
    grid_img.save(output_path)
    print(f"Saved comparison grid to: {output_path}")

    return grid_img


def save_individual_predictions(
    image_path: Path,
    results: List[Dict],
    output_dir: Path,
) -> List[Path]:
    """
    Save individual annotated images for each model prediction.

    Args:
        image_path: Path to original image
        results: List of model results
        output_dir: Directory to save annotated images

    Returns:
        List of paths to saved images
    """
    saved_paths = []
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 128, 0), (128, 0, 255)]

    for idx, result in enumerate(results):
        if not result.get("success", False):
            continue

        model_name = result.get("model", "unknown")
        image_name = image_path.stem

        # Parse bounding boxes
        boxes = parse_bounding_boxes(result.get("response", ""))

        if boxes:
            # Create output filename
            output_filename = f"{image_name}_{model_name.replace('/', '_')}_annotated.jpg"
            output_path = output_dir / output_filename

            # Draw and save
            color = colors[idx % len(colors)]
            draw_bounding_boxes(image_path, boxes, output_path, color=color, input_coordinate_range=1000)
            saved_paths.append(output_path)

            print(f"Saved {model_name} prediction to: {output_path}")

    return saved_paths

"""Visualization module for plotting bounding boxes on images."""

import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont


from dataclasses import dataclass


@dataclass
class BoundingBox:
    """
    Bounding box with normalized coordinates (0-1000 range).
    Coordinates are in image space: x=left-to-right, y=top-to-bottom.
    """
    x1: int  # Left edge
    y1: int  # Top edge
    x2: int  # Right edge
    y2: int  # Bottom edge
    label: str


@dataclass
class PixelBoundingBox:
    """
    Bounding box with pixel coordinates (actual image coordinates).
    Coordinates are in image space: x=left-to-right, y=top-to-bottom.
    """
    x1: int  # Left edge (pixels)
    y1: int  # Top edge (pixels)
    x2: int  # Right edge (pixels)
    y2: int  # Bottom edge (pixels)
    label: str


def transform_to_pixels(
    bbox: BoundingBox,
    image_width: int,
    image_height: int,
    input_range: int = 1000
) -> PixelBoundingBox:
    """
    Transform normalized BoundingBox to PixelBoundingBox.

    Args:
        bbox: Normalized bounding box
        image_width: Image width in pixels
        image_height: Image height in pixels
        input_range: Normalization range (default 1000)

    Returns:
        PixelBoundingBox with actual image coordinates
    """
    # Transform from normalized coordinates to pixel coordinates
    pixel_x1 = int((bbox.x1 / input_range) * image_width)
    pixel_y1 = int((bbox.y1 / input_range) * image_height)
    pixel_x2 = int((bbox.x2 / input_range) * image_width)
    pixel_y2 = int((bbox.y2 / input_range) * image_height)

    # Ensure coordinates are within image bounds
    pixel_x1 = max(0, min(pixel_x1, image_width))
    pixel_y1 = max(0, min(pixel_y1, image_height))
    pixel_x2 = max(0, min(pixel_x2, image_width))
    pixel_y2 = max(0, min(pixel_y2, image_height))

    return PixelBoundingBox(
        x1=pixel_x1,
        y1=pixel_y1,
        x2=pixel_x2,
        y2=pixel_y2,
        label=bbox.label
    )


def parse_bounding_boxes(response_text: str) -> List[BoundingBox]:
    """
    Parse bounding boxes from model response text.
    Assumes LLMs always output in [y1,x1,y2,x2] format.

    Args:
        response_text: Model response containing JSON with bounding boxes

    Returns:
        List of BoundingBox objects
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

        # Parse into BoundingBox objects
        validated_boxes = []
        for box_dict in boxes:

            if isinstance(box_dict, dict) and "point" in box_dict and "label" in box_dict:
                coords = box_dict["point"]
                if isinstance(coords, list) and len(coords) == 2:
                    y, x = coords
                    bbox = BoundingBox(x1=x - 5, y1=y - 5, x2=x + 5, y2=y + 5, label=box_dict["label"])
                    validated_boxes.append(bbox)
            if isinstance(box_dict, dict) and "box_2d" in box_dict and "label" in box_dict:
                coords = box_dict["box_2d"]
                if isinstance(coords, list) and len(coords) == 4:
                    # LLMs always output [y1,x1,y2,x2], convert to x1,y1,x2,y2
                    y1, x1, y2, x2 = coords
                    bbox = BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2, label=box_dict["label"])
                    validated_boxes.append(bbox)

        return validated_boxes

    return []


def draw_bounding_boxes(
    # image_path: Path,
    img: Image,
    boxes: List[BoundingBox],
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
        boxes: List of BoundingBox objects
        output_path: Optional path to save the annotated image
        color: RGB color for boxes (default: red)
        width: Line width for boxes
        font_size: Font size for labels
        input_coordinate_range: Range of input coordinates (default: 1000 for normalized coords)

    Returns:
        PIL Image with bounding boxes drawn
    """
    # Load image
    # img = Image.open(image_path)
    draw = ImageDraw.Draw(img)
    img_width, img_height = img.size

    # Try to load a font, fallback to default if not available
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", font_size)
    except:
        font = ImageFont.load_default()

    # Draw each bounding box
    for bbox in boxes:
        # Transform normalized BoundingBox to pixel coordinates
        pixel_bbox = transform_to_pixels(bbox, img_width, img_height, input_coordinate_range)

        # Draw rectangle
        draw.rectangle([pixel_bbox.x1, pixel_bbox.y1, pixel_bbox.x2, pixel_bbox.y2], outline=color, width=width)

        # Draw label background - make sure label fits within image
        label_y = max(pixel_bbox.y1 - 25, 0)  # Don't go above image
        try:
            label_bbox = draw.textbbox((pixel_bbox.x1, label_y), pixel_bbox.label, font=font)
            draw.rectangle(label_bbox, fill=color)
            # Draw label text
            draw.text((pixel_bbox.x1, label_y), pixel_bbox.label, fill=(255, 255, 255), font=font)
        except:
            # Fallback if textbbox is not available (older PIL versions)
            draw.text((pixel_bbox.x1, label_y), pixel_bbox.label, fill=color, font=font)

    # Save if output path provided
    if output_path:
        img.save(output_path)

    return img

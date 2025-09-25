"""Type definitions for bounding boxes."""

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
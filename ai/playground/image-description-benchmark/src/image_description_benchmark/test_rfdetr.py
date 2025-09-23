#!/usr/bin/env python3
"""RF-DETR object detection script for image processing."""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import supervision as sv
from PIL import Image
from rfdetr import RFDETRBase
from rfdetr.util.coco_classes import COCO_CLASSES


def get_image_paths(use_extracted: bool) -> Tuple[Path, List[Path], Path]:
    """Get image paths and output directory based on mode."""
    if use_extracted:
        # Process all extracted images from the MCAP bag
        bag_dir = (
            Path(__file__).parent.parent.parent.parent.parent.parent
            / "untracked_bags/tracking_me_rosbag2_2025_08_13-13_55_08"
        )
        images_dir = bag_dir / "extracted_images"
        image_paths = list(images_dir.glob("*.jpg"))
        output_dir = bag_dir / "rfdetr_results"
    else:
        # Get all JPG images from the main images folder (excluding subfolders)
        images_dir = Path(__file__).parent / "images"
        image_paths = [p for p in images_dir.glob("*.jpg") if p.is_file()]
        output_dir = images_dir / "rfdetr"

    return images_dir, image_paths, output_dir


def process_image(model: RFDETRBase, image_path: Path, output_dir: Path, threshold: float = 0.5) -> Tuple[int, int]:
    """Process a single image and return detection counts."""
    # Load and process image
    image = Image.open(image_path)
    detections = model.predict(image, threshold=threshold)

    # Create annotated image
    annotated_image = image.copy()
    annotated_image = sv.BoxAnnotator().annotate(annotated_image, detections)
    annotated_image = sv.LabelAnnotator().annotate(annotated_image, detections)

    # Count detections and check for people
    total_detections = len(detections)
    person_detections = 0

    if total_detections > 0:
        class_names = [COCO_CLASSES[detections.class_id[i]] for i in range(len(detections))]
        person_detections = sum(1 for name in class_names if name == "person")

    # Save to main directory (all images)
    output_path = output_dir / f"rfdetr_{image_path.name}"
    annotated_image.save(output_path)

    # If no people detected, also save to no_people subdirectory
    if person_detections == 0:
        no_people_dir = output_dir / "no_people"
        no_people_dir.mkdir(exist_ok=True)
        no_people_path = no_people_dir / f"rfdetr_{image_path.name}"
        annotated_image.save(no_people_path)

    return total_detections, person_detections


def print_detection_summary(detections, image_path: Path):
    """Print detailed detection summary for an image."""
    if len(detections) > 0:
        class_names = [COCO_CLASSES[detections.class_id[i]] for i in range(len(detections))]
        confidences = detections.confidence
        print(f"  Detected {len(detections)} objects:")
        for j, (class_name, confidence) in enumerate(zip(class_names, confidences)):
            print(f"    {j+1}. {class_name} (confidence: {confidence:.3f})")
    else:
        print("  No objects detected")


def main():
    """Main function to run RF-DETR object detection."""
    parser = argparse.ArgumentParser(description="Run RF-DETR object detection on images")
    parser.add_argument(
        "--extracted",
        action="store_true",
        help="Process extracted images from MCAP bag instead of test images",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Detection confidence threshold (default: 0.5)",
    )
    parser.add_argument(
        "--progress-interval",
        type=int,
        default=100,
        help="Print progress every N images for large datasets (default: 100)",
    )

    args = parser.parse_args()

    # Initialize model
    print("Loading RF-DETR model...")
    model = RFDETRBase()

    # Get image paths
    images_dir, image_paths, output_dir = get_image_paths(args.extracted)

    if not images_dir.exists():
        print(f"Error: Source directory not found: {images_dir}")
        return 1

    if not image_paths:
        print(f"No JPEG images found in {images_dir}")
        return 1

    # Create output directory
    output_dir.mkdir(exist_ok=True)

    # Print configuration
    mode = "extracted images from MCAP bag" if args.extracted else "test images"
    print(f"Processing {mode}")
    print(f"Source: {images_dir}")
    print(f"Found {len(image_paths)} images to process")
    print(f"Results will be saved to: {output_dir}")
    print(f"Detection threshold: {args.threshold}")

    # Process images
    total_detections = 0
    person_detections = 0
    errors = 0

    for i, image_path in enumerate(image_paths, 1):
        # Show progress
        show_details = not args.extracted or i % args.progress_interval == 0
        if show_details:
            print(f"\n[{i}/{len(image_paths)}] Processing {image_path.name}...")

        # Process image
        img_detections, img_persons = process_image(model, image_path, output_dir, args.threshold)
        total_detections += img_detections
        person_detections += img_persons

        # Print detailed summary if requested
        if show_details:
            # Load detections again for detailed output (could be optimized)
            image = Image.open(image_path)
            detections = model.predict(image, threshold=args.threshold)
            print_detection_summary(detections, image_path)
            print(f"  Saved: {output_dir.name}/rfdetr_{image_path.name}")

    # Print final statistics
    print(f"\n{'='*60}")
    print("PROCESSING COMPLETE")
    print(f"{'='*60}")
    print(f"Images processed: {len(image_paths)}")
    print(f"Errors: {errors}")
    print(f"Total detections: {total_detections}")
    print(f"Person detections: {person_detections}")
    if len(image_paths) > 0:
        print(f"Average detections per image: {total_detections / len(image_paths):.2f}")
    print(f"Annotated images saved in: {output_dir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

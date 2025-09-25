#!/usr/bin/env python3
"""Multi-model object detection script for image processing."""

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

import numpy as np
import supervision as sv
from PIL import Image
from rfdetr import RFDETRBase
from rfdetr.util.coco_classes import COCO_CLASSES
from ultralytics import YOLO


@dataclass
class DetectionResult:
    """Results from object detection on a single image."""

    image_name: str
    total_detections: int
    person_detections: int
    has_person: bool
    class_names: List[str]
    confidences: List[float]


def get_image_paths(use_extracted: bool, model_name: str) -> Tuple[Path, List[Path], Path]:
    """Get image paths and output directory based on mode."""
    if use_extracted:
        # Process all extracted images from the MCAP bag
        bag_dir = (
            Path(__file__).parent.parent.parent.parent.parent.parent
            / "untracked_bags/tracking_me_rosbag2_2025_08_13-13_55_08"
        )
        images_dir = bag_dir / "extracted_images"
        image_paths = list(images_dir.glob("*.jpg"))
        output_dir = bag_dir / f"{model_name}_results"
    else:
        # Get all JPG images from the main images folder (excluding subfolders)
        images_dir = Path(__file__).parent / "images"
        image_paths = [p for p in images_dir.glob("*.jpg") if p.is_file()]
        output_dir = images_dir / model_name

    return images_dir, image_paths, output_dir


class ModelDetector:
    """Base class for object detection models."""

    def __init__(self, model_name: str):
        self.model_name = model_name
        self.model = None

    def load_model(self):
        """Load the detection model."""
        raise NotImplementedError

    def detect(self, image: Image.Image, threshold: float) -> DetectionResult:
        """Run detection on an image."""
        raise NotImplementedError

    def annotate_image(self, image: Image.Image, result: DetectionResult) -> Image.Image:
        """Create annotated image with detection results."""
        raise NotImplementedError


class RFDETRDetector(ModelDetector):
    """RF-DETR detection model wrapper."""

    def __init__(self):
        super().__init__("rfdetr")

    def load_model(self):
        self.model = RFDETRBase()
        self.model.optimize_for_inference()

    def detect(self, image: Image.Image, threshold: float) -> DetectionResult:
        detections = self.model.predict(image, threshold=threshold)

        total_detections = len(detections)
        person_detections = 0
        class_names = []
        confidences = []

        if total_detections > 0:
            class_names = [COCO_CLASSES[detections.class_id[i]] for i in range(len(detections))]
            confidences = detections.confidence.tolist()
            person_detections = sum(1 for name in class_names if name == "person")

        return DetectionResult(
            image_name="",  # Will be set by caller
            total_detections=total_detections,
            person_detections=person_detections,
            has_person=person_detections > 0,
            class_names=class_names,
            confidences=confidences,
        )

    def annotate_image(self, image: Image.Image, result: DetectionResult) -> Image.Image:
        detections = self.model.predict(image, threshold=0.5)  # Re-run for annotation
        annotated_image = image.copy()
        annotated_image = sv.BoxAnnotator().annotate(annotated_image, detections)
        annotated_image = sv.LabelAnnotator().annotate(annotated_image, detections)
        return annotated_image


class YOLODetector(ModelDetector):
    """YOLO detection model wrapper."""

    def __init__(self, model_version: str):
        super().__init__(f"yolo{model_version}")
        self.model_version = model_version

    def load_model(self):
        if self.model_version == "v8":
            self.model = YOLO("yolov8m.pt")
        elif self.model_version == "v11":
            self.model = YOLO("yolo11m.pt")
        else:
            raise ValueError(f"Unsupported YOLO version: {self.model_version}")

    def detect(self, image: Image.Image, threshold: float) -> DetectionResult:
        results = self.model(image, conf=threshold, verbose=False)

        total_detections = 0
        person_detections = 0
        class_names = []
        confidences = []

        if results and len(results) > 0:
            result = results[0]
            if result.boxes is not None:
                total_detections = len(result.boxes)

                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names[class_id]

                    class_names.append(class_name)
                    confidences.append(confidence)

                    if class_name == "person":
                        person_detections += 1

        return DetectionResult(
            image_name="",  # Will be set by caller
            total_detections=total_detections,
            person_detections=person_detections,
            has_person=person_detections > 0,
            class_names=class_names,
            confidences=confidences,
        )

    def annotate_image(self, image: Image.Image, result: DetectionResult) -> Image.Image:
        results = self.model(image, conf=0.5, verbose=False)
        if results and len(results) > 0:
            return results[0].plot()
        return image.copy()


def process_image_with_model(
    detector: ModelDetector, image_path: Path, output_dir: Path, threshold: float
) -> DetectionResult:
    """Process a single image with a detection model."""
    # Load and process image
    image = Image.open(image_path)
    result = detector.detect(image, threshold)
    result.image_name = image_path.name

    # Create annotated image
    annotated_image = detector.annotate_image(image, result)

    # Convert numpy array to PIL Image if needed
    if isinstance(annotated_image, np.ndarray):
        annotated_image = Image.fromarray(annotated_image)

    # Save to main directory (all images)
    output_path = output_dir / f"{detector.model_name}_{image_path.name}"
    annotated_image.save(output_path)

    # If no people detected, also save to no_people subdirectory
    if not result.has_person:
        no_people_dir = output_dir / "no_people"
        no_people_dir.mkdir(exist_ok=True)
        no_people_path = no_people_dir / f"{detector.model_name}_{image_path.name}"
        annotated_image.save(no_people_path)

    return result


def print_detection_summary(result: DetectionResult):
    """Print detailed detection summary for an image."""
    if result.total_detections > 0:
        print(f"  Detected {result.total_detections} objects:")
        for j, (class_name, confidence) in enumerate(zip(result.class_names, result.confidences)):
            print(f"    {j+1}. {class_name} (confidence: {confidence:.3f})")
    else:
        print("  No objects detected")


def save_comparison_csv(results_by_model: Dict[str, List[DetectionResult]], output_dir: Path):
    """Save CSV comparing person detections across models."""
    csv_path = output_dir / "person_detection_comparison.csv"

    # Get all image names
    all_images = set()
    for model_results in results_by_model.values():
        for result in model_results:
            all_images.add(result.image_name)

    with open(csv_path, "w", newline="") as csvfile:
        fieldnames = ["image_name"]
        for model_name in results_by_model.keys():
            fieldnames.extend(
                [
                    f"{model_name}_total_detections",
                    f"{model_name}_person_detections",
                    f"{model_name}_has_person",
                ]
            )

        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for image_name in sorted(all_images):
            row = {"image_name": image_name}

            for model_name, model_results in results_by_model.items():
                # Find result for this image
                result = next((r for r in model_results if r.image_name == image_name), None)
                if result:
                    row[f"{model_name}_total_detections"] = result.total_detections
                    row[f"{model_name}_person_detections"] = result.person_detections
                    row[f"{model_name}_has_person"] = result.has_person
                else:
                    row[f"{model_name}_total_detections"] = 0
                    row[f"{model_name}_person_detections"] = 0
                    row[f"{model_name}_has_person"] = False

            writer.writerow(row)

    print(f"Comparison CSV saved to: {csv_path}")


def find_disagreeing_images(csv_path: Path) -> List[Dict]:
    """Find images where models disagree on person detection."""
    disagreeing_images = []

    with open(csv_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            # Extract has_person values for each model
            rfdetr_has_person = row["rfdetr_has_person"] == "True"
            yolov8_has_person = row["yolov8_has_person"] == "True"
            yolov11_has_person = row["yolov11_has_person"] == "True"

            # Check if not all models agree
            person_detections = [rfdetr_has_person, yolov8_has_person, yolov11_has_person]
            if not all(person_detections) and any(person_detections):
                disagreeing_images.append(
                    {
                        "image_name": row["image_name"],
                        "rfdetr_has_person": rfdetr_has_person,
                        "yolov8_has_person": yolov8_has_person,
                        "yolov11_has_person": yolov11_has_person,
                    }
                )

    return disagreeing_images


def create_composite_image(image_path: Path, models_results: Dict[str, bool], output_path: Path):
    """Create a composite image showing all three model results side by side."""
    # Load original image
    original_image = Image.open(image_path)

    # Calculate dimensions for composite
    img_width, img_height = original_image.size
    composite_width = img_width * 3 + 40  # 3 images + spacing
    composite_height = img_height + 80  # extra space for labels

    # Create composite image
    composite = Image.new("RGB", (composite_width, composite_height), "white")

    # Model names and their detection paths
    models = ["rfdetr", "yolov8", "yolov11"]
    detection_dir = image_path.parent.parent / "detection_results"

    for i, model in enumerate(models):
        x_offset = i * (img_width + 20)

        # Try to load annotated image, fallback to original
        annotated_path = detection_dir / model / f"{model}_{image_path.name}"
        if annotated_path.exists():
            model_image = Image.open(annotated_path)
        else:
            model_image = original_image.copy()

        # Resize if needed
        if model_image.size != (img_width, img_height):
            model_image = model_image.resize((img_width, img_height))

        # Paste image
        composite.paste(model_image, (x_offset, 60))

        # Add label
        try:
            from PIL import ImageDraw, ImageFont

            draw = ImageDraw.Draw(composite)

            # Model name
            model_text = model.upper()
            # Person detection status
            has_person = models_results.get(f"{model}_has_person", False)
            status_text = "PERSON DETECTED" if has_person else "NO PERSON"

            # Try to use a default font, fallback to basic if not available
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
            except:
                font = ImageFont.load_default()

            # Draw model name
            draw.text((x_offset + 10, 10), model_text, fill="black", font=font)
            # Draw detection status with color coding
            status_color = "green" if has_person else "red"
            draw.text((x_offset + 10, 30), status_text, fill=status_color, font=font)

        except ImportError:
            # PIL ImageDraw/ImageFont not available, skip labels
            pass

    # Save composite image
    composite.save(output_path)


def run_composite_mode(csv_path: Path):
    """Run composite comparison mode."""
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    # Find disagreeing images
    disagreeing_images = find_disagreeing_images(csv_path)

    if not disagreeing_images:
        print("No disagreeing images found. All models agree on person detection.")
        return 0

    print(f"Found {len(disagreeing_images)} images where models disagree:")

    # Create output directory for composites
    composite_dir = csv_path.parent / "composite_comparisons"
    composite_dir.mkdir(exist_ok=True)

    # Source images are in extracted_images directory
    images_dir = csv_path.parent.parent / "extracted_images"

    if not images_dir.exists():
        print(f"Error: Source image directory not found: {images_dir}")
        return 1

    for disagreement in disagreeing_images:
        image_name = disagreement["image_name"]
        print(
            f"  {image_name}: RFDETR={disagreement['rfdetr_has_person']}, "
            f"YOLOv8={disagreement['yolov8_has_person']}, "
            f"YOLOv11={disagreement['yolov11_has_person']}"
        )

        # Get source image path
        source_path = images_dir / image_name

        if not source_path.exists():
            print(f"    Warning: Source image not found: {source_path}")
            continue

        # Create composite
        output_path = composite_dir / f"composite_{image_name}"
        create_composite_image(source_path, disagreement, output_path)
        print(f"    Composite saved: {output_path}")

    print(f"\nComposite images saved to: {composite_dir}")
    return 0


def main():
    """Main function to run multi-model object detection."""
    parser = argparse.ArgumentParser(description="Run multi-model object detection on images")
    parser.add_argument(
        "--extracted",
        action="store_true",
        help="Process extracted images from MCAP bag instead of test images",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        choices=["rfdetr", "yolov8", "yolov11", "all"],
        default=["all"],
        help="Models to run (default: all)",
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
    parser.add_argument(
        "--composite",
        type=str,
        metavar="CSV_FILE",
        help="Create composite comparison images from CSV file for images where models disagree",
    )

    args = parser.parse_args()

    # Handle composite mode
    if args.composite:
        csv_path = Path(args.composite)
        return run_composite_mode(csv_path)

    # Expand "all" models
    if "all" in args.models:
        models_to_run = ["rfdetr", "yolov8", "yolov11"]
    else:
        models_to_run = args.models

    # Initialize detectors
    detectors = {}
    for model_name in models_to_run:
        print(f"Loading {model_name.upper()} model...")
        if model_name == "rfdetr":
            detector = RFDETRDetector()
        elif model_name == "yolov8":
            detector = YOLODetector("v8")
        elif model_name == "yolov11":
            detector = YOLODetector("v11")
        else:
            print(f"Unknown model: {model_name}")
            continue

        detector.load_model()
        detectors[model_name] = detector

    if not detectors:
        print("No valid models specified")
        return 1

    # Get image paths (use first detector for path resolution)
    first_detector = list(detectors.values())[0]
    images_dir, image_paths, base_output_dir = get_image_paths(args.extracted, first_detector.model_name)

    if not images_dir.exists():
        print(f"Error: Source directory not found: {images_dir}")
        return 1

    if not image_paths:
        print(f"No JPEG images found in {images_dir}")
        return 1

    # Create base output directory
    if args.extracted:
        base_output_dir = base_output_dir.parent / "detection_results"
    else:
        base_output_dir = images_dir / "detection_results"
    base_output_dir.mkdir(exist_ok=True)

    # Print configuration
    mode = "extracted images from MCAP bag" if args.extracted else "test images"
    print(f"Processing {mode}")
    print(f"Source: {images_dir}")
    print(f"Found {len(image_paths)} images to process")
    print(f"Models: {', '.join(models_to_run)}")
    print(f"Results will be saved to: {base_output_dir}")
    print(f"Detection threshold: {args.threshold}")

    # Process images with each model
    results_by_model = {}

    for model_name, detector in detectors.items():
        print(f"\n{'='*60}")
        print(f"PROCESSING WITH {model_name.upper()}")
        print(f"{'='*60}")

        model_output_dir = base_output_dir / model_name
        model_output_dir.mkdir(exist_ok=True)

        model_results = []
        total_detections = 0
        person_detections = 0

        for i, image_path in enumerate(image_paths, 1):
            # Show progress
            show_details = not args.extracted or i % args.progress_interval == 0
            if show_details:
                print(f"\n[{i}/{len(image_paths)}] Processing {image_path.name} with {model_name}...")

            # Process image
            result = process_image_with_model(detector, image_path, model_output_dir, args.threshold)
            model_results.append(result)

            total_detections += result.total_detections
            person_detections += result.person_detections

            # Print detailed summary if requested
            if show_details:
                print_detection_summary(result)
                print(f"  Saved: {model_output_dir.name}/{model_name}_{image_path.name}")

        results_by_model[model_name] = model_results

        # Print model summary
        print(f"\n{model_name.upper()} Summary:")
        print(f"Total detections: {total_detections}")
        print(f"Person detections: {person_detections}")
        images_with_people = sum(1 for r in model_results if r.has_person)
        print(f"Images with people: {images_with_people}/{len(model_results)}")

    # Save comparison CSV
    save_comparison_csv(results_by_model, base_output_dir)

    # Print final comparison
    print(f"\n{'='*60}")
    print("FINAL COMPARISON")
    print(f"{'='*60}")
    print(f"Images processed: {len(image_paths)}")

    for model_name, model_results in results_by_model.items():
        images_with_people = sum(1 for r in model_results if r.has_person)
        total_people = sum(r.person_detections for r in model_results)
        print(f"{model_name.upper()}: {images_with_people} images with people, {total_people} total person detections")

    print(f"\nResults saved in: {base_output_dir}")
    print("Comparison CSV: person_detection_comparison.csv")

    return 0


if __name__ == "__main__":
    sys.exit(main())

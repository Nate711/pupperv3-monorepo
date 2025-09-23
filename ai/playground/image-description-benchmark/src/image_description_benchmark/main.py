#!/usr/bin/env python3
"""Main entry point for the image description benchmark."""

from pathlib import Path
from .benchmark import ImageDescriptionBenchmark


def main():
    """Run the image description benchmark with default settings."""
    # Configuration
    models = [
        # "gpt-5",
        # "gpt-5-mini",
        # "gpt-4o",
        "gpt-realtime",  # Uses WebSocket API
    ]

    # Get images from hailo config (relative to monorepo root)
    # Assuming we run from ai/playground/image-description-benchmark
    image_dir = Path("../../../ros2_ws/src/hailo/config").resolve()

    if not image_dir.exists():
        print(f"Error: Image directory not found at {image_dir}")
        print("Please run from ai/playground/image-description-benchmark directory")
        return 1

    image_paths = list(image_dir.glob("*.png"))

    if not image_paths:
        print(f"No PNG images found in {image_dir}")
        return 1

    print(f"Found {len(image_paths)} images to benchmark")
    print(f"Testing models: {', '.join(models)}")
    print("Note: gpt-realtime uses WebSocket API and is primarily designed for audio/voice")
    print()

    # Run benchmark
    benchmark = ImageDescriptionBenchmark()

    prompt = "Describe the location of the person in the image in two numbers. x: 0 to 1 (left to right) and y: 0 to 1 (top to bottom) in 2 decimals. Example output: '0.45, 0.75'"

    benchmark.run(models=models, image_paths=image_paths, prompt=prompt, delay=0.5, include_realtime=True)

    # Show results
    benchmark.print_summary()
    benchmark.save_results(Path.cwd())

    return 0


if __name__ == "__main__":
    exit(main())

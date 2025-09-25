#!/usr/bin/env python3
"""Main entry point for the combined image description benchmark."""

from pathlib import Path
import sys
import os

# Add src to Python path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from image_description_benchmark.benchmark import ImageDescriptionBenchmark, GeminiImageBenchmark


def main():
    """Run the image description benchmark with both OpenAI and Gemini models."""
    # Configuration for OpenAI models
    openai_models = [
        "gpt-5",
        "gpt-5-mini",
        "gpt-4o",
    ]

    # Configuration for Gemini models
    gemini_models = [
        "gemini-2.5-flash",
    ]

    # Get images from local images folder
    image_dir = Path(__file__).parent / "src" / "image_description_benchmark" / "images"

    if not image_dir.exists():
        print(f"Error: Image directory not found at {image_dir}")
        print("Please ensure the images folder exists in the package directory")
        return 1

    image_paths = list(image_dir.glob("*.jpg"))

    if not image_paths:
        print(f"No JPG images found in {image_dir}")
        return 1

    print(f"Found {len(image_paths)} images to benchmark")
    print(f"Testing OpenAI models: {', '.join(openai_models)}")
    print(f"Testing Gemini models: {', '.join(gemini_models)}")
    print()

    prompt = "Describe the location of the person in the image in two numbers. x: 0 to 1 (left to right) and y: 0 to 1 (top to bottom) in 2 decimals. Example output: '0.45, 0.75'"

    # Run OpenAI benchmark
    print("=" * 60)
    print("RUNNING OPENAI BENCHMARKS")
    print("=" * 60)
    openai_benchmark = ImageDescriptionBenchmark()
    openai_benchmark.run(models=openai_models, image_paths=image_paths, prompt=prompt, delay=0.5)
    openai_benchmark.print_summary()
    openai_benchmark.save_results(Path.cwd())

    print("\n" * 2)

    # Run Gemini benchmark
    print("=" * 60)
    print("RUNNING GEMINI BENCHMARKS")
    print("=" * 60)
    gemini_benchmark = GeminiImageBenchmark()
    gemini_benchmark.run(models=gemini_models, image_paths=image_paths, prompt=prompt, delay=0.5)
    gemini_benchmark.print_summary()
    gemini_benchmark.save_results(Path.cwd())

    return 0


if __name__ == "__main__":
    exit(main())

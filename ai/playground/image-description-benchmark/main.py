#!/usr/bin/env python3
"""Main entry point for the combined image description benchmark."""

import sys
from pathlib import Path

# Add src to Python path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from image_description_benchmark import BenchmarkRunner


def main():
    """Run the image description benchmark with both OpenAI and Gemini models."""
    try:
        # Initialize runner with default configuration
        runner = BenchmarkRunner()

        # Run all benchmarks
        runner.run_all()

        return 0

    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

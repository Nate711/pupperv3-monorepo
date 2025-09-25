#!/usr/bin/env python3
"""Main entry point for the image description benchmark."""

import argparse
import sys
from pathlib import Path

from image_description_benchmark.runner import BenchmarkRunner


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Run image description benchmarks on OpenAI and Gemini models")

    parser.add_argument(
        "--image-dir",
        type=Path,
        help="Directory containing test images (defaults to package images folder)",
    )

    parser.add_argument(
        "--prompt",
        type=str,
        help="Custom prompt for image description",
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to save results (defaults to current directory)",
    )

    parser.add_argument(
        "--delay",
        type=float,
        default=0.5,
        help="Delay between API calls in seconds (default: 0.5)",
    )

    parser.add_argument(
        "--openai-only",
        action="store_true",
        help="Run only OpenAI benchmarks",
    )

    parser.add_argument(
        "--gemini-only",
        action="store_true",
        help="Run only Gemini benchmarks",
    )

    parser.add_argument(
        "--models",
        nargs="+",
        help="Specific models to test (overrides defaults)",
    )

    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip visualization generation",
    )

    return parser.parse_args()


def main():
    """Run the image description benchmark with command line arguments."""
    args = parse_arguments()

    # Handle mutual exclusivity
    if args.openai_only and args.gemini_only:
        print("Error: Cannot specify both --openai-only and --gemini-only")
        return 1

    # Configure models based on arguments
    openai_models = None
    gemini_models = None

    if args.openai_only:
        gemini_models = []
        # Keep openai_models as None to use defaults, unless specific models provided
    elif args.gemini_only:
        openai_models = []
        # Keep gemini_models as None to use defaults, unless specific models provided

    # Override with specific models if provided
    if args.models:
        if args.openai_only:
            openai_models = args.models
            gemini_models = []  # Ensure gemini is disabled
        elif args.gemini_only:
            gemini_models = args.models
            openai_models = []  # Ensure openai is disabled
        else:
            # If no specific provider is specified, try to determine from model names
            openai_models = [m for m in args.models if "gpt" in m.lower()]
            gemini_models = [m for m in args.models if "gemini" in m.lower()]

    try:
        # Initialize and run benchmark runner
        runner = BenchmarkRunner(
            image_dir=args.image_dir,
            openai_models=openai_models,
            gemini_models=gemini_models,
        )

        runner.run_all(
            prompt=args.prompt,
            delay=args.delay,
            output_dir=args.output_dir,
            generate_visualizations=not args.no_viz,
        )

        return 0

    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by user")
        return 130
    except Exception as e:
        print(f"Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

"""Utility functions for image description benchmarks."""

import base64
import json
import os
from pathlib import Path
from typing import List, Optional
from datetime import datetime

import pandas as pd
from dotenv import load_dotenv


def load_api_key(key_name: str, env_file: str = ".env.local") -> str:
    """
    Load API key from environment file.

    Args:
        key_name: Name of the environment variable
        env_file: Path to environment file

    Returns:
        API key string

    Raises:
        ValueError: If API key not found
    """
    load_dotenv(env_file, override=True)
    api_key = os.getenv(key_name)
    if not api_key:
        raise ValueError(f"{key_name} not found in {env_file} file in current directory")
    return api_key


def encode_image_base64(image_path: Path) -> str:
    """
    Encode image to base64 string.

    Args:
        image_path: Path to image file

    Returns:
        Base64 encoded image string
    """
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


def load_image_bytes(image_path: Path) -> bytes:
    """
    Load image as bytes.

    Args:
        image_path: Path to image file

    Returns:
        Image bytes
    """
    with open(image_path, "rb") as f:
        return f.read()


def get_image_paths(image_dir: Path) -> List[Path]:
    """
    Get all JPG image paths from a directory.

    Args:
        image_dir: Directory containing images

    Returns:
        List of image paths
    """
    if not image_dir.exists():
        raise FileNotFoundError(f"Image directory not found at {image_dir}")

    image_paths = list(image_dir.glob("*.jpg"))
    if not image_paths:
        raise ValueError(f"No JPG images found in {image_dir}")

    return image_paths


def save_results_to_files(
    results: List[dict], output_dir: Optional[Path] = None, prefix: str = "benchmark"
) -> tuple[Path, Path]:
    """
    Save benchmark results to JSON and CSV files.

    Args:
        results: List of benchmark result dictionaries
        output_dir: Directory to save results (defaults to current directory)
        prefix: Prefix for output filenames

    Returns:
        Tuple of (json_file_path, csv_file_path)
    """
    if not results:
        raise ValueError("No results to save")

    output_dir = output_dir or Path.cwd()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Save JSON
    json_file = output_dir / f"{prefix}_results_{timestamp}.json"
    with open(json_file, "w") as f:
        json.dump(results, f, indent=2)

    # Save CSV
    df = pd.DataFrame(results)
    csv_file = output_dir / f"{prefix}_results_{timestamp}.csv"
    df.to_csv(csv_file, index=False)

    return json_file, csv_file


def print_benchmark_statistics(results: List[dict], title: str = "BENCHMARK RESULTS"):
    """
    Print summary statistics of benchmark results.

    Args:
        results: List of benchmark result dictionaries
        title: Title to display
    """
    if not results:
        print("No results to summarize")
        return

    df = pd.DataFrame(results)

    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)

    # Group by model and calculate statistics
    successful_results = df[df["success"]]
    if not successful_results.empty:
        model_stats = successful_results.groupby("model")["latency"].agg(["count", "mean", "std", "min", "max"])

        print("\nLatency Statistics by Model (seconds):")
        print(model_stats.to_string())

        # Average latency per image
        image_stats = successful_results.groupby("image")["latency"].mean().sort_values()
        print("\nAverage Latency by Image (seconds):")
        for image, latency in image_stats.items():
            print(f"  {image}: {latency:.2f}s")

        # Overall statistics
        print(f"\nOverall Statistics:")
        print(f"  Total successful tests: {len(successful_results)}")
        print(f"  Total failed tests: {len(df) - len(successful_results)}")
        print(f"  Average latency: {successful_results['latency'].mean():.2f}s")
        print(f"  Median latency: {successful_results['latency'].median():.2f}s")
    else:
        print("No successful results to analyze")

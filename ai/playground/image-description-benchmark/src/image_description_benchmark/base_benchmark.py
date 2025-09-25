"""Base benchmark class with shared functionality."""

import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List, Optional

from .utils import print_benchmark_statistics, save_results_to_files


class BaseBenchmark(ABC):
    """Abstract base class for image description benchmarks."""

    def __init__(self):
        """Initialize benchmark with empty results list."""
        self.results: List[Dict] = []

    @abstractmethod
    def benchmark_model(self, model: str, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single model on a single image.

        Args:
            model: Model name/identifier
            image_path: Path to image file
            prompt: Prompt to use for image description

        Returns:
            Dictionary containing benchmark results
        """
        pass

    def run(
        self,
        models: List[str],
        image_paths: List[Path],
        prompt: str,
        delay: float = 0.5,
    ) -> List[Dict]:
        """
        Run benchmarks across all models and images.

        Args:
            models: List of model names to test
            image_paths: List of image paths to test
            prompt: Prompt to use for image description
            delay: Delay between tests to avoid rate limiting

        Returns:
            List of benchmark result dictionaries
        """
        self.results = []

        total_tests = len(models) * len(image_paths)
        current_test = 0

        print(f"Starting {self.__class__.__name__} with {len(models)} models")
        print(f"Images to test: {len(image_paths)}")
        print(f"Total tests to run: {total_tests}\n")

        # Test all models
        for model in models:
            for image_path in image_paths:
                current_test += 1
                print(f"[{current_test}/{total_tests}] Testing {model} on {image_path.name}...")

                result = self.benchmark_model(model, image_path, prompt)
                self.results.append(result)

                if result["success"]:
                    print(f"  ✓ Completed in {result['latency']:.2f}s")
                else:
                    print(f"  ✗ Failed: {result['error']}")

                time.sleep(delay)

        return self.results

    def print_summary(self, title: Optional[str] = None):
        """
        Print summary statistics of the benchmark.

        Args:
            title: Optional custom title for the summary
        """
        if title is None:
            title = f"{self.__class__.__name__} RESULTS"
        print_benchmark_statistics(self.results, title)

    def save_results(self, output_dir: Optional[Path] = None, prefix: Optional[str] = None):
        """
        Save results to JSON and CSV files.

        Args:
            output_dir: Directory to save results (defaults to current directory)
            prefix: Prefix for output filenames

        Returns:
            Tuple of (json_file_path, csv_file_path)
        """
        if prefix is None:
            prefix = self.__class__.__name__.lower().replace("benchmark", "")

        json_file, csv_file = save_results_to_files(self.results, output_dir, prefix)
        print(f"\nDetailed results saved to: {json_file}")
        print(f"CSV results saved to: {csv_file}")
        return json_file, csv_file

"""Benchmark runner module for orchestrating multiple benchmarks."""

from pathlib import Path
from typing import List, Optional

from .config import (
    BENCHMARK_DELAY,
    DEFAULT_PROMPT,
    GEMINI_MODELS,
    OPENAI_MODELS,
)
from .gemini_benchmark import GeminiBenchmark
from .openai_benchmark import OpenAIBenchmark
from .utils import get_image_paths


class BenchmarkRunner:
    """Orchestrates running multiple benchmarks."""

    def __init__(
        self,
        image_dir: Optional[Path] = None,
        openai_models: Optional[List[str]] = None,
        gemini_models: Optional[List[str]] = None,
    ):
        """
        Initialize benchmark runner.

        Args:
            image_dir: Directory containing test images
            openai_models: List of OpenAI models to test (defaults to config)
            gemini_models: List of Gemini models to test (defaults to config)
        """
        self.openai_models = openai_models or OPENAI_MODELS
        self.gemini_models = gemini_models or GEMINI_MODELS

        # Set default image directory
        if image_dir is None:
            image_dir = Path(__file__).parent / "images"
        self.image_dir = image_dir

    def run_all(
        self,
        prompt: Optional[str] = None,
        delay: float = BENCHMARK_DELAY,
        output_dir: Optional[Path] = None,
    ):
        """
        Run all configured benchmarks.

        Args:
            prompt: Prompt to use for image description (defaults to config)
            delay: Delay between API calls
            output_dir: Directory to save results

        Returns:
            Dictionary containing results from all benchmarks
        """
        prompt = prompt or DEFAULT_PROMPT
        output_dir = output_dir or Path.cwd()

        # Get image paths
        image_paths = get_image_paths(self.image_dir)
        print(f"Found {len(image_paths)} images to benchmark")

        results = {}

        # Run OpenAI benchmarks if models are configured
        if self.openai_models:
            results["openai"] = self.run_openai_benchmark(image_paths, prompt, delay, output_dir)

        # Add spacing between benchmarks
        if self.openai_models and self.gemini_models:
            print("\n" * 2)

        # Run Gemini benchmarks if models are configured
        if self.gemini_models:
            results["gemini"] = self.run_gemini_benchmark(image_paths, prompt, delay, output_dir)

        return results

    def run_openai_benchmark(
        self,
        image_paths: List[Path],
        prompt: str,
        delay: float,
        output_dir: Path,
    ) -> dict:
        """
        Run OpenAI benchmark.

        Args:
            image_paths: List of image paths to test
            prompt: Prompt to use
            delay: Delay between API calls
            output_dir: Directory to save results

        Returns:
            Dictionary with benchmark results
        """
        print("=" * 60)
        print("RUNNING OPENAI BENCHMARKS")
        print("=" * 60)
        print(f"Testing models: {', '.join(self.openai_models)}")
        print()

        benchmark = OpenAIBenchmark()
        results = benchmark.run(
            models=self.openai_models,
            image_paths=image_paths,
            prompt=prompt,
            delay=delay,
        )

        benchmark.print_summary("OPENAI BENCHMARK RESULTS")
        files = benchmark.save_results(output_dir, "openai")

        return {
            "results": results,
            "files": files,
        }

    def run_gemini_benchmark(
        self,
        image_paths: List[Path],
        prompt: str,
        delay: float,
        output_dir: Path,
    ) -> dict:
        """
        Run Gemini benchmark.

        Args:
            image_paths: List of image paths to test
            prompt: Prompt to use
            delay: Delay between API calls
            output_dir: Directory to save results

        Returns:
            Dictionary with benchmark results
        """
        print("=" * 60)
        print("RUNNING GEMINI BENCHMARKS")
        print("=" * 60)
        print(f"Testing models: {', '.join(self.gemini_models)}")
        print()

        benchmark = GeminiBenchmark()
        results = benchmark.run(
            models=self.gemini_models,
            image_paths=image_paths,
            prompt=prompt,
            delay=delay,
        )

        benchmark.print_summary("GEMINI BENCHMARK RESULTS")
        files = benchmark.save_results(output_dir, "gemini")

        return {
            "results": results,
            "files": files,
        }

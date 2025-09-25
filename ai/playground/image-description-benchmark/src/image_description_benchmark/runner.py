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
from .result_processor import ResultProcessor
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
        self.openai_models = openai_models if openai_models is not None else OPENAI_MODELS
        self.gemini_models = gemini_models if gemini_models is not None else GEMINI_MODELS

        # Set default image directory
        if image_dir is None:
            image_dir = Path(__file__).parent / "images"
        self.image_dir = image_dir

    def run_all(
        self,
        prompt: Optional[str] = None,
        delay: float = BENCHMARK_DELAY,
        output_dir: Optional[Path] = None,
        generate_visualizations: bool = True,
    ):
        """
        Run all configured benchmarks.

        Args:
            prompt: Prompt to use for image description (defaults to config)
            delay: Delay between API calls
            output_dir: Directory to save results
            generate_visualizations: Whether to generate visualization outputs

        Returns:
            Dictionary containing results from all benchmarks
        """
        prompt = prompt or DEFAULT_PROMPT
        output_dir = output_dir or Path.cwd()

        # Get image paths
        image_paths = get_image_paths(self.image_dir)
        print(f"Found {len(image_paths)} images to benchmark")

        results = {}
        all_results = []

        # Run OpenAI benchmarks if models are configured
        if self.openai_models:
            openai_data = self.run_openai_benchmark(image_paths, prompt, delay)
            results["openai"] = openai_data
            all_results.extend(openai_data["results"])

        # Add spacing between benchmarks
        if self.openai_models and self.gemini_models:
            print("\n" * 2)

        # Run Gemini benchmarks if models are configured
        if self.gemini_models:
            gemini_data = self.run_gemini_benchmark(image_paths, prompt, delay)
            results["gemini"] = gemini_data
            all_results.extend(gemini_data["results"])

        # Process results and generate visualizations
        if all_results and generate_visualizations:
            print("\n" * 2)
            print("=" * 60)
            print("PROCESSING RESULTS AND GENERATING VISUALIZATIONS")
            print("=" * 60)

            processor = ResultProcessor(all_results, self.image_dir)

            # Print bounding box analysis
            processor.print_bounding_box_summary()

            # Generate visualizations in dedicated folder
            viz_dir = output_dir / "visualizations"
            viz_paths = processor.generate_visualizations(viz_dir)
            results["visualizations"] = viz_paths

        return results

    def run_openai_benchmark(
        self,
        image_paths: List[Path],
        prompt: str,
        delay: float,
    ) -> dict:
        """
        Run OpenAI benchmark.

        Args:
            image_paths: List of image paths to test
            prompt: Prompt to use
            delay: Delay between API calls

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

        return {
            "results": results,
        }

    def run_gemini_benchmark(
        self,
        image_paths: List[Path],
        prompt: str,
        delay: float,
    ) -> dict:
        """
        Run Gemini benchmark.

        Args:
            image_paths: List of image paths to test
            prompt: Prompt to use
            delay: Delay between API calls

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

        return {
            "results": results,
        }

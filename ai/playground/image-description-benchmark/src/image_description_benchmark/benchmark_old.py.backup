"""Core benchmarking functionality."""

import base64
import time
import os
from pathlib import Path
from typing import Dict
import json
from datetime import datetime

from dotenv import load_dotenv
from openai import OpenAI
import pandas as pd
from google import genai
from google.genai import types

bounding_box_system_instructions = """
    Return bounding boxes as a JSON array with labels. Never return masks or code fencing. Limit to 25 objects.
    If an object is present multiple times, name them according to their unique characteristic (colors, size, position, unique characteristics, etc..).
      """


class ImageDescriptionBenchmark:
    """Benchmark OpenAI models on image description tasks."""

    def __init__(self, client: OpenAI = None):
        """Initialize benchmark with OpenAI client."""
        if not client:
            # Load .env.local file from current working directory
            load_dotenv(".env.local", override=True)

            # Get API key from loaded environment
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY not found in .env.local file in current directory")

            self.client = OpenAI(api_key=api_key)
        else:
            self.client = client

        self.results = []

    def encode_image(self, image_path: Path) -> str:
        """Encode image to base64 string."""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    def benchmark_model(self, model: str, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single model on a single image.
        Returns timing and response information.
        """
        base64_image = self.encode_image(image_path)

        start_time = time.perf_counter()

        try:
            if "gpt-5" in model:
                kwargs = {
                    "reasoning": {"effort": "minimal"},
                }
            else:
                kwargs = {}

            response = self.client.responses.create(
                model=model,
                input=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "input_text", "text": prompt},
                            {"type": "input_image", "image_url": f"data:image/jpeg;base64,{base64_image}"},
                        ],
                    }
                ],
                **kwargs,
            )

            end_time = time.perf_counter()
            latency = end_time - start_time

            # Extract response text from output array
            response_text = None
            for item in response.output:
                # Look for ResponseOutputMessage type which contains the text
                if hasattr(item, "content") and item.content:
                    for content_item in item.content:
                        if hasattr(content_item, "text"):
                            response_text = content_item.text
                            break
                    if response_text:
                        break

            if not response_text:
                raise ValueError(f"No text response found in API output for {model}")

            # Extract token usage
            tokens_used = response.usage.total_tokens if response.usage else None

            return {
                "model": model,
                "image": image_path.name,
                "latency": latency,
                "response": response_text,
                "tokens_used": tokens_used,
                "success": True,
                "error": None,
            }

        except Exception as e:
            end_time = time.perf_counter()
            latency = end_time - start_time

            return {
                "model": model,
                "image": image_path.name,
                "latency": latency,
                "response": None,
                "tokens_used": None,
                "success": False,
                "error": str(e),
            }

    def run(self, models: list[str], image_paths: list[Path], prompt: str, delay: float = 0.5):
        """
        Run benchmarks across all models and images.

        Args:
            models: List of model names to test
            image_paths: List of image paths to test
            prompt: Prompt to use for image description
            delay: Delay between tests to avoid rate limiting
        """
        self.results = []

        total_tests = len(models) * len(image_paths)
        current_test = 0

        print(f"Starting benchmark with {len(models)} models")
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

    def print_summary(self):
        """Print summary statistics of the benchmark."""
        if not self.results:
            print("No results to summarize")
            return

        df = pd.DataFrame(self.results)

        print("\n" + "=" * 60)
        print("BENCHMARK RESULTS")
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

    def save_results(self, output_dir: Path = None):
        """Save results to JSON and CSV files."""
        if not self.results:
            print("No results to save")
            return

        output_dir = output_dir or Path.cwd()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save JSON
        json_file = output_dir / f"benchmark_results_{timestamp}.json"
        with open(json_file, "w") as f:
            json.dump(self.results, f, indent=2)
        print(f"\nDetailed results saved to: {json_file}")

        # Save CSV
        df = pd.DataFrame(self.results)
        csv_file = output_dir / f"benchmark_results_{timestamp}.csv"
        df.to_csv(csv_file, index=False)
        print(f"CSV results saved to: {csv_file}")

        return json_file, csv_file


safety_settings = [
    types.SafetySetting(
        category="HARM_CATEGORY_DANGEROUS_CONTENT",
        threshold="BLOCK_ONLY_HIGH",
    ),
]


class GeminiImageBenchmark:
    """Benchmark Gemini models on image description tasks."""

    def __init__(self, client: genai.Client = None):
        """Initialize benchmark with Gemini client."""
        if not client:
            # Load .env.local file from current working directory
            load_dotenv(".env.local", override=True)

            # Get API key from loaded environment
            api_key = os.getenv("GOOGLE_API_KEY")
            if not api_key:
                raise ValueError("GOOGLE_API_KEY not found in .env.local file in current directory")

            self.client = genai.Client(api_key=api_key)
        else:
            self.client = client

        self.results = []

    def benchmark_model(self, model: str, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single model on a single image.
        Returns timing and response information.
        """
        start_time = time.perf_counter()

        try:
            with open(image_path, "rb") as f:
                image_bytes = f.read()

            response = self.client.models.generate_content(
                model=model,
                contents=[
                    types.Part.from_bytes(
                        data=image_bytes,
                        mime_type="image/jpeg",
                    ),
                    prompt,
                ],
                config=types.GenerateContentConfig(
                    system_instruction=bounding_box_system_instructions,
                    temperature=0.5,
                    safety_settings=safety_settings,
                    thinking_config=types.ThinkingConfig(thinking_budget=0),
                ),
            )

            end_time = time.perf_counter()
            latency = end_time - start_time

            print(f"Response: {response.text}")

            return {
                "model": model,
                "image": image_path.name,
                "latency": latency,
                "response": response.text,
                "tokens_used": (
                    getattr(response, "usage", {}).get("total_tokens", None) if hasattr(response, "usage") else None
                ),
                "success": True,
                "error": None,
            }

        except Exception as e:
            end_time = time.perf_counter()
            latency = end_time - start_time

            return {
                "model": model,
                "image": image_path.name,
                "latency": latency,
                "response": None,
                "tokens_used": None,
                "success": False,
                "error": str(e),
            }

    def run(self, models: list[str], image_paths: list[Path], prompt: str, delay: float = 0.5):
        """
        Run benchmarks across all models and images.

        Args:
            models: List of model names to test
            image_paths: List of image paths to test
            prompt: Prompt to use for image description
            delay: Delay between tests to avoid rate limiting
        """
        self.results = []

        total_tests = len(models) * len(image_paths)
        current_test = 0

        print(f"Starting Gemini benchmark with {len(models)} models")
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

    def print_summary(self):
        """Print summary statistics of the benchmark."""
        if not self.results:
            print("No results to summarize")
            return

        df = pd.DataFrame(self.results)

        print("\n" + "=" * 60)
        print("GEMINI BENCHMARK RESULTS")
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

    def save_results(self, output_dir: Path = None):
        """Save results to JSON and CSV files."""
        if not self.results:
            print("No results to save")
            return

        output_dir = output_dir or Path.cwd()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save JSON
        json_file = output_dir / f"gemini_benchmark_results_{timestamp}.json"
        with open(json_file, "w") as f:
            json.dump(self.results, f, indent=2)
        print(f"\nDetailed results saved to: {json_file}")

        # Save CSV
        df = pd.DataFrame(self.results)
        csv_file = output_dir / f"gemini_benchmark_results_{timestamp}.csv"
        df.to_csv(csv_file, index=False)
        print(f"CSV results saved to: {csv_file}")

        return json_file, csv_file

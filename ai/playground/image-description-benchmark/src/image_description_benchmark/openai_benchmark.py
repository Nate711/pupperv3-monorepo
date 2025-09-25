"""OpenAI-specific benchmark implementation."""

import time
from pathlib import Path
from typing import Dict, Optional

from openai import OpenAI

from .base_benchmark import BaseBenchmark
from .utils import encode_image_base64, load_api_key
from .config import BOUNDING_BOX_SYSTEM_INSTRUCTIONS


class OpenAIBenchmark(BaseBenchmark):
    """Benchmark OpenAI models on image description tasks."""

    def __init__(self, client: Optional[OpenAI] = None):
        """
        Initialize benchmark with OpenAI client.

        Args:
            client: Optional pre-configured OpenAI client
        """
        super().__init__()

        if not client:
            api_key = load_api_key("OPENAI_API_KEY")
            self.client = OpenAI(api_key=api_key)
        else:
            self.client = client

    def benchmark_model(self, model: str, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single OpenAI model on a single image.

        Args:
            model: OpenAI model name
            image_path: Path to image file
            prompt: Prompt to use for image description

        Returns:
            Dictionary containing benchmark results
        """
        base64_image = encode_image_base64(image_path)
        start_time = time.perf_counter()

        try:
            # Add reasoning parameter for GPT-5 models
            kwargs = {}
            if "gpt-5" in model:
                kwargs["reasoning"] = {"effort": "minimal"}

            response = self.client.responses.create(
                model=model,
                instructions=BOUNDING_BOX_SYSTEM_INSTRUCTIONS,
                input=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "input_text", "text": prompt},
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{base64_image}",
                            },
                        ],
                    }
                ],
                **kwargs,
            )

            end_time = time.perf_counter()
            latency = end_time - start_time

            # Extract response text from output array
            response_text = self._extract_response_text(response)
            print(f"Model: {model}, Image: {image_path.name}, Response: {response_text}")

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

    def _extract_response_text(self, response) -> Optional[str]:
        """
        Extract text from OpenAI API response.

        Args:
            response: OpenAI API response object

        Returns:
            Extracted text or None if not found
        """
        for item in response.output:
            if hasattr(item, "content") and item.content:
                for content_item in item.content:
                    if hasattr(content_item, "text"):
                        return content_item.text
        return None

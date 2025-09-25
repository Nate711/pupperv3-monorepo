"""Gemini-specific benchmark implementation."""

import time
from pathlib import Path
from typing import Dict, Optional

from google import genai
from google.genai import types

from .base_benchmark import BaseBenchmark
from .config import BOUNDING_BOX_SYSTEM_INSTRUCTIONS, GEMINI_SAFETY_SETTINGS
from .utils import load_api_key, load_image_bytes


class GeminiBenchmark(BaseBenchmark):
    """Benchmark Gemini models on image description tasks."""

    def __init__(self, client: Optional[genai.Client] = None):
        """
        Initialize benchmark with Gemini client.

        Args:
            client: Optional pre-configured Gemini client
        """
        super().__init__()

        if not client:
            api_key = load_api_key("GOOGLE_API_KEY")
            self.client = genai.Client(api_key=api_key)
        else:
            self.client = client

    def benchmark_model(self, model: str, image_path: Path, prompt: str) -> Dict:
        """
        Benchmark a single Gemini model on a single image.

        Args:
            model: Gemini model name
            image_path: Path to image file
            prompt: Prompt to use for image description

        Returns:
            Dictionary containing benchmark results
        """
        start_time = time.perf_counter()

        try:
            image_bytes = load_image_bytes(image_path)

            response = self.client.models.generate_content(
                model=model,
                contents=[
                    types.Part.from_bytes(
                        data=image_bytes,
                        mime_type="image/jpeg",
                    ),
                    # prompt,
                    "Detect persons in the image. Output a json list where each entry contains the 2D bounding box in 'box_2d' and a text label in 'label'. Do not output point detections or box_3d detections",
                    # "Point out 5 points on the torso of each person in the image. Output a json list where each entry contains a point in 'point' and a text label in 'label'. Do not output box_2d or box_3d.",
                ],
                config=types.GenerateContentConfig(
                    # system_instruction=BOUNDING_BOX_SYSTEM_INSTRUCTIONS,
                    temperature=0.5,
                    # safety_settings=GEMINI_SAFETY_SETTINGS,
                    thinking_config=types.ThinkingConfig(thinking_budget=0),
                ),
            )

            end_time = time.perf_counter()
            latency = end_time - start_time

            # Optional: Log response for debugging
            print(f"Response: {response.text}")

            tokens_used = self._extract_token_usage(response)

            return {
                "model": model,
                "image": image_path.name,
                "latency": latency,
                "response": response.text,
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

    def _extract_token_usage(self, response) -> Optional[int]:
        """
        Extract token usage from Gemini API response.

        Args:
            response: Gemini API response object

        Returns:
            Total tokens used or None if not available
        """
        if hasattr(response, "usage"):
            usage_dict = getattr(response, "usage", {})
            if isinstance(usage_dict, dict):
                return usage_dict.get("total_tokens", None)
        return None

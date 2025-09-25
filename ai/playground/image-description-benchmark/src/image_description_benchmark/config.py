"""Configuration module for image description benchmarks."""

from typing import List
from google.genai import types

BOUNDING_BOX_SYSTEM_INSTRUCTIONS = """
Return bounding boxes as a JSON array with labels. Never return masks or code fencing. Limit to 25 objects.
If an object is present multiple times, name them according to their unique characteristic (colors, size, position, unique characteristics, etc..).

Example output:
```json
[
  {"box_2d": [46, 246, 385, 526], "label": "light blue sock with cat face on left"},
  {"box_2d": [233, 661, 650, 862], "label": "light blue and grey sock with cat face on right"}
]
```
"""

DEFAULT_PROMPT = "Detect the location of the person in the image in x and y coordinates (both normalized to 0 to 1000)"

OPENAI_MODELS = [
    "gpt-5",
    "gpt-5-mini",
    "gpt-4o",
]

GEMINI_MODELS = ["gemini-2.5-flash", "gemini-2.5-flash-lite"]

GEMINI_SAFETY_SETTINGS = [
    types.SafetySetting(
        category="HARM_CATEGORY_DANGEROUS_CONTENT",
        threshold="BLOCK_ONLY_HIGH",
    ),
]

BENCHMARK_DELAY = 0.5  # Delay between API calls to avoid rate limiting

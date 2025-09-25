"""Configuration module for image description benchmarks."""

from google.genai import types

BOUNDING_BOX_SYSTEM_INSTRUCTIONS = """
Output a json list where each entry contains the 2D bounding box in "box_2d" and a text label in "label". The box_2d coordinates should be normalized to a 0-1000 scale and in order [y1, x1, y2, x2].

Example output:
```json
[
  {"box_2d": [46, 246, 385, 526], "label": "light blue sock with cat face on left"},
  {"box_2d": [233, 661, 650, 862], "label": "light blue and grey sock with cat face on right"}
]
```
"""

DEFAULT_PROMPT = "Detect the location of the person in the image."

OPENAI_MODELS = [
    "gpt-5",
    "gpt-5-mini",
    "gpt-4o",
]

GEMINI_MODELS = ["gemini-2.5-flash", "gemini-2.5-flash-lite", "gemini-robotics-er-1.5-preview"]

GEMINI_SAFETY_SETTINGS = [
    types.SafetySetting(
        category="HARM_CATEGORY_DANGEROUS_CONTENT",
        threshold="BLOCK_ONLY_HIGH",
    ),
]

BENCHMARK_DELAY = 0.5  # Delay between API calls to avoid rate limiting

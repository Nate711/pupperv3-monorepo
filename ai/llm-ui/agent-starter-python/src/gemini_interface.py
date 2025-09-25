from google import genai
from google.genai import types
from PIL import Image
from pathlib import Path

import os
from dotenv import load_dotenv
import gemini_utils


load_dotenv(".env.local", override=True)
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

SYSTEM_INSTRUCTIONS = """
Return points of interest as a JSON array with labels. Never return masks or code fencing. Limit to 25 objects.
If an object is present multiple times, name them according to their unique characteristic (colors, size, position, unique characteristics, etc..).

Example output:
```json
[
  {"point": [527, 508], "label": "reachable position on the ground to find the bathroom"}
]
```
"""

MODEL_NAME = "gemini-robotics-er-1.5-preview"
client = genai.Client(api_key=GOOGLE_API_KEY)


def analyze_camera_image(prompt: str, image: Image) -> tuple[bool, str]:
    response = client.models.generate_content(
        model=MODEL_NAME,
        contents=[image, prompt],
        config=types.GenerateContentConfig(
            temperature=0.5,
            system_instruction=SYSTEM_INSTRUCTIONS,
            # thinking_config=types.ThinkingConfig(thinking_budget=1024),
        ),
    )
    return response.text

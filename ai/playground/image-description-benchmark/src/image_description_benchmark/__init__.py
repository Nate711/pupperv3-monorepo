"""Image description benchmark package."""

from .base_benchmark import BaseBenchmark
from .config import (
    BENCHMARK_DELAY,
    BOUNDING_BOX_SYSTEM_INSTRUCTIONS,
    DEFAULT_PROMPT,
    GEMINI_MODELS,
    GEMINI_SAFETY_SETTINGS,
    OPENAI_MODELS,
)
from .gemini_benchmark import GeminiBenchmark
from .openai_benchmark import OpenAIBenchmark
from .runner import BenchmarkRunner
from .utils import (
    encode_image_base64,
    get_image_paths,
    load_api_key,
    load_image_bytes,
    print_benchmark_statistics,
    save_results_to_files,
)

__version__ = "0.1.0"

__all__ = [
    # Classes
    "BaseBenchmark",
    "OpenAIBenchmark",
    "GeminiBenchmark",
    "BenchmarkRunner",
    # Config constants
    "OPENAI_MODELS",
    "GEMINI_MODELS",
    "DEFAULT_PROMPT",
    "BENCHMARK_DELAY",
    "BOUNDING_BOX_SYSTEM_INSTRUCTIONS",
    "GEMINI_SAFETY_SETTINGS",
    # Utility functions
    "load_api_key",
    "encode_image_base64",
    "load_image_bytes",
    "get_image_paths",
    "save_results_to_files",
    "print_benchmark_statistics",
]

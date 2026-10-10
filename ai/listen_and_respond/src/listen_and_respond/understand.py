"""Step 2: understanding.

Sends each transcribed utterance, with a few lines of recent history, to Claude Haiku and
gets back a validated Interpretation: command / conversation / background, the commands in
the unified format from commands.py, and a reply for conversation.
"""

import os
from collections import deque
from pathlib import Path
from typing import Literal

import anthropic
from pydantic import BaseModel

from listen_and_respond.commands import Command, describe_commands

MODEL = "claude-haiku-5-5"
SYSTEM_PROMPT = (Path(__file__).with_name("system_prompt.md")).read_text().replace("{commands}", describe_commands())


class Interpretation(BaseModel):
    intent: Literal["command", "conversation", "background"]
    commands: list[Command]
    reply: str | None
    language: Literal["en", "zh"]


class Understanding:
    def __init__(self, history_lines: int = 8):
        if not os.environ.get("ANTHROPIC_API_KEY"):
            raise RuntimeError(
                "ANTHROPIC_API_KEY is not set. Put ANTHROPIC_API_KEY=... in ai/listen_and_respond/.env"
            )
        self.client = anthropic.Anthropic()
        # Recent exchanges, so the model can tell a continuing conversation from background talk.
        self.history: deque[str] = deque(maxlen=history_lines)

    def interpret(self, text: str, lang: str) -> Interpretation:
        history = "\n".join(self.history) if self.history else "(nothing yet)"
        response = self.client.messages.parse(
            model=MODEL,
            max_tokens=1024,
            output_config={"effort": "low"},  # short classification; keeps latency down
            system=SYSTEM_PROMPT,
            messages=[
                {
                    "role": "user",
                    "content": f"Recent history:\n{history}\n\nNew utterance [{lang}]: {text}",
                }
            ],
            output_format=Interpretation,
        )
        if response.stop_reason != "end_turn":
            raise RuntimeError(f"Haiku stopped with {response.stop_reason!r}: {response.stop_details}")
        result = response.parsed_output

        self.history.append(f"Heard [{lang}]: {text}  -> {result.intent}")
        if result.reply:
            self.history.append(f"ZhuangZi replied: {result.reply}")
        return result

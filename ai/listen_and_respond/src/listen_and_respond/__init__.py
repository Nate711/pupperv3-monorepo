import argparse
import json
import time
from pathlib import Path

from dotenv import load_dotenv

PROJECT_DIR = Path(__file__).resolve().parents[2]
COMMAND_LOG = PROJECT_DIR / "logs" / "commands.jsonl"


def log_commands(text: str, lang: str, interpretation) -> None:
    COMMAND_LOG.parent.mkdir(exist_ok=True)
    entry = {
        "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "heard": text,
        "lang": lang,
        "commands": [c.model_dump() for c in interpretation.commands],
    }
    with COMMAND_LOG.open("a") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def handle(understanding, text: str, lang: str) -> None:
    start = time.monotonic()
    result = understanding.interpret(text, lang)
    took = f"({time.monotonic() - start:.2f}s)"
    if result.intent == "command":
        log_commands(text, lang, result)
        steps = ", ".join(json.dumps(c.model_dump(exclude_none=True)) for c in result.commands)
        print(f"  -> COMMAND {steps} {took}", flush=True)
    elif result.intent == "conversation":
        print(f"  -> REPLY [{result.language}] {result.reply} {took}", flush=True)
    else:
        print(f"  -> background, ignored {took}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="ZhuangZi listen-and-respond")
    parser.add_argument("--text", action="store_true", help="type utterances instead of using the mic")
    parser.add_argument("--stt-only", action="store_true", help="only transcribe; skip understanding")
    args = parser.parse_args()
    if args.text and args.stt_only:
        parser.error("--text needs understanding; drop --stt-only")

    load_dotenv(PROJECT_DIR / ".env")
    understanding = None
    if not args.stt_only:
        from listen_and_respond.understand import Understanding

        understanding = Understanding()

    try:
        if args.text:
            print("Type what you would say (prefix zh: for Chinese); Ctrl+D to stop.", flush=True)
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                lang, text = ("zh", line[3:].strip()) if line.startswith("zh:") else ("en", line)
                handle(understanding, text, lang)
            return

        from listen_and_respond.stt import SpeechToText

        print("Loading models...", flush=True)
        stt = SpeechToText()
        print("Listening. Speak English or Chinese; Ctrl+C to stop.", flush=True)
        for u in stt.listen():
            clip = "  CLIPPED" if u.peak >= 0.99 else ""
            print(
                f"[{u.lang or '?'}] {u.text}   "
                f"({u.duration_s:.1f}s speech, {u.decode_s:.2f}s decode, {u.emotion}/{u.event}){clip}",
                flush=True,
            )
            if understanding:
                handle(understanding, u.text, u.lang)
    except KeyboardInterrupt:
        print("Stopped.")

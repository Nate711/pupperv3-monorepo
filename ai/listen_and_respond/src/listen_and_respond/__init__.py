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


def execute(robot, commands) -> None:
    """Runs the steps in order and stops at the first one that fails."""
    for i, command in enumerate(commands):
        ok, what = robot.execute(command)
        if not ok:
            skipped = len(commands) - i - 1
            note = f" (skipped the {skipped} remaining step(s))" if skipped else ""
            print(f"  !! {command.action} FAILED: {what}{note}", flush=True)
            return
        print(f"  => {command.action}: {what}", flush=True)


def handle(understanding, robot, text: str, lang: str) -> None:
    start = time.monotonic()
    result = understanding.interpret(text, lang)
    took = f"({time.monotonic() - start:.2f}s)"
    if result.intent == "command":
        log_commands(text, lang, result)
        steps = ", ".join(json.dumps(c.model_dump(exclude_none=True)) for c in result.commands)
        print(f"  -> COMMAND {steps} {took}", flush=True)
        if robot:
            execute(robot, result.commands)
    elif result.intent == "conversation":
        print(f"  -> REPLY [{result.language}] {result.reply} {took}", flush=True)
    else:
        print(f"  -> background, ignored {took}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="ZhuangZi listen-and-respond")
    parser.add_argument("--text", action="store_true", help="type utterances instead of using the mic")
    parser.add_argument("--stt-only", action="store_true", help="only transcribe; skip understanding")
    parser.add_argument("--no-robot", action="store_true", help="log commands without sending them to the robot")
    args = parser.parse_args()
    if args.text and args.stt_only:
        parser.error("--text needs understanding; drop --stt-only")

    load_dotenv(PROJECT_DIR / ".env")
    understanding = None
    if not args.stt_only:
        from listen_and_respond.understand import Understanding

        understanding = Understanding()

    robot = None
    if understanding and not args.no_robot:
        from listen_and_respond.robot import Robot

        robot = Robot()

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
                handle(understanding, robot, text, lang)
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
                handle(understanding, robot, u.text, u.lang)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if robot:
            robot.close()

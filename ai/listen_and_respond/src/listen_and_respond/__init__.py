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


def dispatch(runner, commands) -> None:
    """A stop anywhere in the utterance interrupts what is running; steps after the last stop
    ("stop, then turn left") are queued as usual."""
    stops = [i for i, c in enumerate(commands) if c.action == "stop"]
    if stops:
        runner.stop()
        commands = commands[stops[-1] + 1 :]
    if commands:
        runner.submit(commands)


def handle(understanding, runner, text: str, lang: str) -> None:
    if runner:
        from listen_and_respond.runner import heard_stop_word

        if heard_stop_word(text):
            runner.stop()
            print("  -> STOP (stop word heard; halting before understanding)", flush=True)
    start = time.monotonic()
    result = understanding.interpret(text, lang)
    took = f"({time.monotonic() - start:.2f}s)"
    if result.intent == "command":
        log_commands(text, lang, result)
        steps = ", ".join(json.dumps(c.model_dump(exclude_none=True)) for c in result.commands)
        print(f"  -> COMMAND {steps} {took}", flush=True)
        if runner:
            dispatch(runner, result.commands)
    elif result.intent == "conversation":
        print(f"  -> REPLY [{result.language}] {result.reply} {took}", flush=True)
    else:
        print(f"  -> background, ignored {took}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="ZhuangZi listen-and-respond")
    parser.add_argument("--text", action="store_true", help="type utterances instead of using the mic")
    parser.add_argument("--stt-only", action="store_true", help="only transcribe; skip understanding")
    parser.add_argument("--no-robot", action="store_true", help="log commands without sending them to the robot")
    parser.add_argument(
        "--fence-cm",
        type=float,
        help="testing: refuse moves that could leave a square of this side (cm) around the start position",
    )
    args = parser.parse_args()
    if args.text and args.stt_only:
        parser.error("--text needs understanding; drop --stt-only")

    load_dotenv(PROJECT_DIR / ".env")
    understanding = None
    if not args.stt_only:
        from listen_and_respond.understand import Understanding

        understanding = Understanding()

    robot = runner = None
    if understanding and not args.no_robot:
        from listen_and_respond.robot import Robot
        from listen_and_respond.runner import CommandRunner

        robot = Robot(fence_side_m=None if args.fence_cm is None else args.fence_cm / 100)
        runner = CommandRunner(robot)

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
                handle(understanding, runner, text, lang)
            if runner:
                runner.wait_until_idle()  # let typed commands finish before exiting
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
                handle(understanding, runner, u.text, u.lang)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if runner:
            runner.close()
        if robot:
            robot.close()

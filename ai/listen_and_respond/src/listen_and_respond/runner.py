"""Runs commands on the robot in a background thread, so the voice loop keeps listening while
ZhuangZi moves, and lets a spoken "stop" interrupt whatever is running.

Each utterance's commands are one batch, run in order after any batches before it. stop()
bumps a generation counter: the running command sees it within one control step and halts,
and batches queued before the stop are dropped. Batches submitted after it run normally.

Two paths reach stop():
- heard_stop_word() on the raw transcript, before Haiku, so the robot halts without waiting
  ~1 s for the model. It matches any "stop" in the text ("don't stop" too); a false stop only
  halts the robot.
- Haiku returning a `stop` command, which also covers phrasings the word list misses.
"""

import queue
import re
import threading

from listen_and_respond.commands import Command
from listen_and_respond.robot import Outcome, Robot

STOP_WORDS_EN = re.compile(r"\b(stop|halt|freeze)\b", re.IGNORECASE)
STOP_WORDS_ZH = ("停", "别动")


def heard_stop_word(text: str) -> bool:
    return bool(STOP_WORDS_EN.search(text)) or any(word in text for word in STOP_WORDS_ZH)


class CommandRunner:
    def __init__(self, robot: Robot):
        self.robot = robot
        self._generation = 0
        self._generation_lock = threading.Lock()
        self._batches: queue.Queue[tuple[int, list[Command]] | None] = queue.Queue()
        self._thread = threading.Thread(target=self._work, daemon=True)
        self._thread.start()

    def submit(self, commands: list[Command]) -> None:
        with self._generation_lock:
            generation = self._generation
        self._batches.put((generation, commands))

    def stop(self) -> None:
        with self._generation_lock:
            self._generation += 1

    def wait_until_idle(self) -> None:
        self._batches.join()

    def close(self) -> None:
        """Stops anything running, drops the queue, and ends the worker."""
        self.stop()
        self._batches.put(None)
        self._thread.join()

    def _work(self) -> None:
        while True:
            item = self._batches.get()
            if item is None:
                self._batches.task_done()
                return
            generation, commands = item
            self._run(generation, commands)
            self._batches.task_done()

    def _run(self, generation: int, commands: list[Command]) -> None:
        def stopped() -> bool:
            return self._generation != generation

        for i, command in enumerate(commands):
            rest = len(commands) - i - 1
            dropped = f" (dropped the {rest} remaining step(s))" if rest else ""
            if stopped():
                print(f"  .. {command.action} not started: stop{dropped}", flush=True)
                return
            outcome, what = self.robot.execute(command, stopped)
            if outcome is Outcome.STOPPED:
                print(f"  .. {command.action} STOPPED: {what}{dropped}", flush=True)
                return
            if outcome is Outcome.FAILED:
                print(f"  !! {command.action} FAILED: {what}{dropped}", flush=True)
                return
            print(f"  => {command.action}: {what}", flush=True)

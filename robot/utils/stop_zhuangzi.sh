#!/bin/bash
# Stops what start_zhuangzi.sh started: voice first (so it can't send more commands), then the
# robot stack, which deactivates the motors on the way out. Both get Ctrl+C, the same clean
# shutdown as pressing it in their panes.
set -uo pipefail

SESSION=zhuangzi

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "No tmux session '$SESSION'; nothing to stop."
  exit 0
fi

echo "Stopping voice..."
tmux send-keys -t "$SESSION":0.1 C-c
for _ in $(seq 1 10); do
  pgrep -f "bin/listen-and-respond" >/dev/null || break
  sleep 1
done
if pgrep -f "bin/listen-and-respond" >/dev/null; then
  echo "WARNING: voice is still running after 10 s." >&2
fi

echo "Stopping the robot stack (this takes up to a minute)..."
tmux send-keys -t "$SESSION":0.0 C-c
for _ in $(seq 1 60); do
  pgrep -f "ros2_control_node" >/dev/null || break
  sleep 1
done
if pgrep -f "ros2_control_node" >/dev/null; then
  echo "WARNING: ros2_control_node is still running after 60 s; the tmux session is left open so you can look." >&2
  exit 1
fi

# The panes wait for Enter after their program exits, so the session is normally still here.
if tmux has-session -t "$SESSION" 2>/dev/null; then tmux kill-session -t "$SESSION"; fi
echo "ZhuangZi stopped."

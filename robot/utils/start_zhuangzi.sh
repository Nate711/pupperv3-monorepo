#!/bin/bash
# Starts ZhuangZi: the robot stack (robot.sh) and the voice stack (listen_and_respond), each in
# its own pane of the tmux session "zhuangzi". Voice starts once controller_manager is up.
# Any arguments go to listen-and-respond, e.g. `start_zhuangzi.sh --fence-cm 50`.
# Stop everything with stop_zhuangzi.sh. Detach with Ctrl+B then D; reattach with
# `tmux attach -t zhuangzi`.
set -euo pipefail

SESSION=zhuangzi
REPO="$HOME/pupperv3-monorepo"
VOICE_ARGS="$*"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "ZhuangZi is already running (tmux session '$SESSION'). Attaching; run stop_zhuangzi.sh to stop it."
  if [ -t 1 ]; then exec tmux attach -t "$SESSION"; fi
  exit 0
fi

# A second controller_manager (another stack, or a test stand-in) would answer the spawners and
# the voice commands in place of the real one.
set +u  # the ROS setup script reads unset variables
source /opt/ros/jazzy/setup.bash
set -u
services=$(ros2 service list 2>/dev/null || true)
if grep -q "^/controller_manager/" <<<"$services"; then
  echo "ERROR: a controller_manager is already running on this ROS network; stop it first." >&2
  ros2 node list 2>/dev/null >&2
  exit 1
fi

if ! pactl list short sources | grep -q C920; then
  echo "WARNING: the C920 webcam mic is not connected; voice will listen to the wrong input." >&2
fi

# Top pane: robot stack. Homing starts as soon as it launches.
tmux new-session -d -s "$SESSION" -n zhuangzi "$REPO/robot/utils/robot.sh; echo 'Robot stack exited.'; read"

# Bottom pane: voice, after controller_manager's switch service appears.
tmux split-window -v -t "$SESSION" "
  source /opt/ros/jazzy/setup.bash
  echo 'Waiting for the robot stack (controller_manager)...'
  until ros2 service type /controller_manager/switch_controller >/dev/null 2>&1; do sleep 1; done
  cd '$REPO/ai/listen_and_respond'
  uv run listen-and-respond $VOICE_ARGS
  echo 'Voice exited.'; read"

tmux select-pane -t "$SESSION":0.1
echo "Started tmux session '$SESSION': robot stack on top, voice below."
if [ -t 1 ]; then exec tmux attach -t "$SESSION"; fi

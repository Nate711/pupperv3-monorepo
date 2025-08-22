#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${1:-pupper-dev}"

# Sanity check
if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Please install tmux and rerun." >&2
  exit 1
fi

# If session exists, just attach to it
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  exec tmux attach -t "$SESSION_NAME"
fi

# --- Paths ---
WS_DIR="$HOME/pupperv3-monorepo/ros2_ws/src/llm_websocket_server"
UI_DIR="$HOME/pupperv3-monorepo/ai/llm-ui/live-audio"

# --- Create session & panes ---
# Pane 0: WebSocket server (left, largest)
tmux new-session -d -s "$SESSION_NAME" -n dev -c "$WS_DIR"
tmux send-keys  -t "$SESSION_NAME":0.0 "python3 llm_websocket_server/websocket_server.py" C-m

# Split to create right column (~33%), leaving left ~67% for the websocket server
tmux split-window -h -t "$SESSION_NAME":0.0 -p 33 -c "$UI_DIR"

# Right column is now pane 1. Split it again to make three vertical panes (two smaller on the right).
tmux split-window -h -t "$SESSION_NAME":0.1 -p 50 -c "$HOME"

# Pane 1 (middle): npm run dev
tmux send-keys -t "$SESSION_NAME":0.1 "npm run dev" C-m

# Pane 2 (right): Chromium fullscreen to localhost:5173 on DISPLAY=:0
tmux send-keys -t "$SESSION_NAME":0.2 'DISPLAY=:0 chromium-browser --new-window "http://localhost:5173" --start-fullscreen' C-m

# Focus the big left pane (websocket server) and attach
tmux select-pane -t "$SESSION_NAME":0.0
exec tmux attach -t "$SESSION_NAME"


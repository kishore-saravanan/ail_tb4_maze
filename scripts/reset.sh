#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ -f .runtime/results/infrastructure.pid ]]; then
  challenge_pid=$(cat .runtime/results/infrastructure.pid)
  if [[ -r /proc/$challenge_pid/cmdline ]] && tr '\0' ' ' < "/proc/$challenge_pid/cmdline" | rg -q 'ros2 launch ail_tb4_maze challenge.launch.py'; then
    kill -INT "$challenge_pid"
    for attempt in {1..100}; do
      if ! kill -0 "$challenge_pid" 2>/dev/null; then break; fi
      sleep 0.2
    done
    if kill -0 "$challenge_pid" 2>/dev/null; then
      echo 'The previous launch is still stopping; wait before resetting.' >&2
      exit 1
    fi
  fi
fi
exec scripts/launch.sh "$@"

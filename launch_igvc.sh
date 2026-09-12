#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SIM_DIR="$ROOT/ws/src/cev-ackermann-sim"
TIMING_FILE="$ROOT/astar_timing.csv"

source /opt/ros/humble/setup.bash
source "$ROOT/ws/install/setup.bash"
set -u

cleanup() {
  [[ -n "${ASTAR_PID:-}" ]] && kill "$ASTAR_PID" 2>/dev/null || true
  [[ -n "${VISUALIZER_PID:-}" ]] && kill "$VISUALIZER_PID" 2>/dev/null || true
  [[ -n "${ADAPTER_PID:-}" ]] && kill "$ADAPTER_PID" 2>/dev/null || true
  pkill -TERM -f '^python3 .*/costmap_adapter.py' 2>/dev/null || true
  pkill -TERM -f '^python3 .*/path_visualizer.py' 2>/dev/null || true
  pkill -TERM -f '.*/target/debug/a_star$' 2>/dev/null || true
  if [[ -s "$TIMING_FILE" ]]; then
    printf '\nA* timing summary:\n'
    python3 "$SIM_DIR/timing_report.py" "$TIMING_FILE" || true
  fi
}
trap cleanup EXIT INT TERM

# Prevent a prior launch from leaving a second fixed or rolling publisher alive.
pkill -TERM -f '^python3 .*/costmap_adapter.py' 2>/dev/null || true
pkill -TERM -f '^python3 .*/path_visualizer.py' 2>/dev/null || true
pkill -TERM -f '.*/target/debug/a_star$' 2>/dev/null || true

ROLLING_WINDOW=false
if [[ "${1:-}" == "--rolling-window" ]]; then
  ROLLING_WINDOW=true
fi

# Each launch produces an independent timing table.
: > "$TIMING_FILE"

python3 "$SIM_DIR/costmap_adapter.py" \
  --ros-args -p robot_radius_m:=0.15 -p unknown_is_obstacle:=false \
  -p rolling_window:="$ROLLING_WINDOW" -p window_width_m:=3.0 -p window_height_m:=3.0 &
ADAPTER_PID=$!

python3 "$SIM_DIR/path_visualizer.py" &
VISUALIZER_PID=$!

RUSTFLAGS="-L native=$ROOT/install/test_msgs/lib" \
  pixi run cargo run --manifest-path "$ROOT/costmap_planner_rs/Cargo.toml" --bin a_star &
ASTAR_PID=$!

python3 "$SIM_DIR/sim.py" "$SIM_DIR/maps/igvc.json"
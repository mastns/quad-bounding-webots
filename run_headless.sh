#!/bin/bash
# Run a converted world without a display (private Xvfb), optionally recording a movie.
#
#   ./run_headless.sh WORLD.wbt [SECONDS] [MOVIE.mp4]
#
#   SECONDS   simulated time to run (default 20)
#   MOVIE     if given, the 3D view is rendered and recorded to this file (1280x720, real-time pace);
#             without it the run is in fast mode with no rendering.
# The controllers' console goes to the terminal and their logs to controllers/<name>/data/*.csv.
# Env: WEBOTS_HOME (default ~/webots), QUADSIM_MOVIE_SIZE (default 1280x720),
#      QUADSIM_VIEW_ZOOM (movie camera distance factor, e.g. 2.5; default 1 = the world's camera),
#      QUADSIM_AT_STOP / QUADSIM_KEYBOARD (see README).
set -euo pipefail
world=$(realpath "${1:?usage: run_headless.sh WORLD.wbt [SECONDS] [MOVIE.mp4]}")
seconds=${2:-20}
movie=${3:+$(realpath -m "$3")}
export WEBOTS_HOME=${WEBOTS_HOME:-$HOME/webots}
here=$(cd "$(dirname "$0")" && pwd)

# Xvfb: system one if installed, else the copy unpacked in ~/.local/opt/xvfb
XVFB=$(command -v Xvfb || echo "$HOME/.local/opt/xvfb/usr/bin/Xvfb")
display=90
while [ -e /tmp/.X11-unix/X$display ] || [ -e /tmp/.X$display-lock ]; do display=$((display + 1)); done
"$XVFB" :$display -screen 0 1600x900x24 -nolisten tcp >/dev/null 2>&1 &
xvfb_pid=$!

# temporary copy of the world with the recorder/timer supervisor, next to the original (relative paths)
project=$(dirname "$(dirname "$world")")
mkdir -p "$project/controllers/movie_recorder"
cp "$here/tools/movie_recorder.py" "$project/controllers/movie_recorder/movie_recorder.py"
tmp_world="$(dirname "$world")/.headless_$(basename "$world")"
cp "$world" "$tmp_world"
printf 'Robot {\n  name "movie recorder"\n  controller "movie_recorder"\n  supervisor TRUE\n}\n' >> "$tmp_world"
cleanup() {
  rm -f "$tmp_world" "$(dirname "$tmp_world")/.$(basename "$tmp_world" .wbt).wbproj"
  kill "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 1
mode=(--mode=fast --no-rendering)
[ -n "$movie" ] && mode=(--mode=realtime)
DISPLAY=:$display QUADSIM_MOVIE="$movie" QUADSIM_SECONDS="$seconds" QUADSIM_AT_STOP=${QUADSIM_AT_STOP:-continue} \
  "$WEBOTS_HOME/webots" --batch --stdout --stderr --minimize --port=$((30000 + $$ % 20000)) "${mode[@]}" "$tmp_world"

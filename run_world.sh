#!/bin/bash
# Open a converted world in the Webots R2025a GUI (needs a display: this machine's desktop,
# ssh -X, or a remote desktop).
#
#   ./run_world.sh [WORLD.wbt]      default: Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct_showcase.wbt
here=$(cd "$(dirname "$0")" && pwd)
export WEBOTS_HOME=${WEBOTS_HOME:-$HOME/webots}
exec "$WEBOTS_HOME/webots" "${1:-$here/Webots_FINAL_2/worlds/SUPERVISOR_webots743_Oct_showcase.wbt}"

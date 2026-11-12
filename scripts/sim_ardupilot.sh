#!/usr/bin/env bash
# Terminal 2: start ArduCopter SITL connected to Gazebo.
#   scripts/sim_ardupilot.sh              # with MAVProxy console + map windows
#   scripts/sim_ardupilot.sh --no-console # plain MAVProxy prompt only
#   scripts/sim_ardupilot.sh --headless   # no prompt, no windows (for scripted runs)
# Add -w once to wipe saved parameters back to defaults.
# MAVROS connects separately on tcp:5762 (SITL's SERIAL1), like the Pi uses TELEM2 on the real drone.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/../setup/env.sh"
source "$HOME/venv-ardupilot/bin/activate"
cd "$HOME/ardupilot"
ARGS=(--console --map)
case "$1" in
  --no-console) ARGS=(); shift ;;
  --headless)   ARGS=(--mavproxy-args="--non-interactive"); shift ;;
esac
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild "${ARGS[@]}" "$@"

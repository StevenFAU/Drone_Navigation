#!/usr/bin/env bash
# Starts the whole sim in one terminal: Gazebo, ArduCopter SITL, MAVROS and the Gazebo-ROS bridge.
#   scripts/start_sim.sh              # Gazebo with its window
#   scripts/start_sim.sh --headless   # no Gazebo window (faster)
#   scripts/start_sim.sh --noise      # GPS + barometer noise (sim/params/noise.parm)
#   scripts/start_sim.sh --gps-denied # no GPS: optical flow + rangefinder, like the T27 indoors
#   WORLD=iris_runway.sdf scripts/start_sim.sh   # another world (default: aruco_world.sdf)
# Ctrl-C here, or closing the Gazebo window, stops them all.
# Each program's output goes to logs/sim_*.log.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$DIR/../logs"
mkdir -p "$LOG"
source "$DIR/../setup/env.sh"

# A leftover sim (SITL, MAVROS, bridge) from an earlier run steals ports and makes this one hang
# at "waiting for MAVROS". Refuse to start until it's gone.
LEFTOVER=$(pgrep -f 'build/sitl/bin/arducopter|gz-sim-main|mavros_node|parameter_bridge /camera' | xargs)
if [ -n "$LEFTOVER" ]; then
  echo "An earlier sim is still running:"
  ps -o pid,etime,cmd -p "${LEFTOVER// /,}" | cut -c1-110
  echo
  echo "Stop it first (Ctrl-C in its terminal), or:  kill $LEFTOVER"
  exit 1
fi

GZ_ARG=""
# -w: wipe ArduPilot's saved parameters every start, so options don't carry over between runs.
AP_ARGS=(--headless -w)
for arg in "$@"; do
  case "$arg" in
    --headless)   GZ_ARG="--headless" ;;
    --noise)      AP_ARGS+=(--add-param-file="$DIR/../sim/params/noise.parm") ;;
    --gps-denied) AP_ARGS+=(--add-param-file="$DIR/../sim/params/gps_denied.parm") ;;
    *) echo "unknown option: $arg (use --headless, --noise, --gps-denied)"; exit 2 ;;
  esac
done

PGIDS=()
start() {  # start <name> <command...>: run in its own process group so we can stop all of it
  local name=$1; shift
  setsid "$@" > "$LOG/sim_$name.log" 2>&1 &
  PGIDS+=($!)
  echo "started $name (log: logs/sim_$name.log)"
}

stop_all() {
  trap - INT TERM EXIT
  echo; echo "stopping sim..."
  for pg in "${PGIDS[@]}"; do kill -TERM -- "-$pg" 2>/dev/null; done
  sleep 2
  for pg in "${PGIDS[@]}"; do kill -KILL -- "-$pg" 2>/dev/null; done
  echo "sim stopped"
}
trap stop_all INT TERM EXIT

start gazebo   "$DIR/sim_gazebo.sh" $GZ_ARG
sleep 5
start ardupilot "$DIR/sim_ardupilot.sh" "${AP_ARGS[@]}"
sleep 10
start mavros   "$DIR/mavros.sh"
start bridge   "$DIR/gz_bridge.sh"

echo "waiting for MAVROS to connect to ArduPilot..."
for _ in $(seq 1 90); do
  if grep -q "Got HEARTBEAT, connected" "$LOG/sim_mavros.log" 2>/dev/null; then
    echo
    echo "READY. In another terminal tab (in the drone_mission folder) run:"
    echo "  source setup/env.sh && ros2 run drone_mission mission"
    echo
    echo "Leave this tab open. Ctrl-C here stops the sim."
    break
  fi
  sleep 2
done

wait -n   # returns as soon as any of them exits (e.g. Gazebo window closed)
echo "one of the sim programs exited - check logs/sim_*.log"

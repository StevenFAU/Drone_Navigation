#!/usr/bin/env bash
# Starts the whole sim in one terminal: Gazebo, ArduCopter SITL and MAVROS.
#   scripts/start_sim.sh              # Gazebo with its window
#   scripts/start_sim.sh --headless   # no Gazebo window (faster)
# Ctrl-C here, or closing the Gazebo window, stops all three.
# Each program's output goes to logs/sim_*.log.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$DIR/../logs"
mkdir -p "$LOG"
source "$DIR/../setup/env.sh"

GZ_ARG=""
[ "$1" = "--headless" ] && GZ_ARG="--headless"

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
start ardupilot "$DIR/sim_ardupilot.sh" --headless
sleep 10
start mavros   "$DIR/mavros.sh"

echo "waiting for MAVROS to connect to ArduPilot..."
for _ in $(seq 1 90); do
  if grep -q "Got HEARTBEAT, connected" "$LOG/sim_mavros.log" 2>/dev/null; then
    echo
    echo "READY. In another terminal tab (in the drone_mission folder) run:"
    echo "  source setup/env.sh && ros2 run drone_mission hello_flight"
    echo
    echo "Leave this tab open. Ctrl-C here stops the sim."
    break
  fi
  sleep 2
done

wait -n   # returns as soon as any of the three exits (e.g. Gazebo window closed)
echo "one of the sim programs exited - check logs/sim_*.log"

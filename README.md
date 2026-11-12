# Drone_Navigation: waypoints + ArUco marker arrival

Simulation setup for the waypoint/marker project. It matches the real T27 drone's
flight software: **ArduCopter 4.6.3** (not PX4), controlled through **MAVROS on ROS 2**.

| Piece | Version | Location |
| --- | --- | --- |
| Ubuntu | 26.04 | |
| ROS 2 | Lyrical | `/opt/ros/lyrical` |
| Gazebo | Jetty (from ROS `ros_gz` vendor packages) | |
| ArduPilot SITL | Copter-4.6.3 | `~/ardupilot` |
| ArduPilot Gazebo plugin | main | `~/ardupilot_gazebo` |
| This project's ROS 2 code | | `ros2_ws/src/drone_mission` |

## One-time setup

On a fresh Ubuntu 26.04 machine, clone this repo, then:

```bash
setup/01_install_system.sh   # downloads ArduPilot + plugin, installs ROS 2 etc.; needs sudo; ~20-40 min
setup/02_build_sim.sh        # no sudo; ~15-30 min
```

With Claude Code on that machine, open this folder and ask it to "set up this project".
`CLAUDE.md` tells it the steps and the problems hit on the first machine.

## Running the sim

Easiest: one terminal tab runs all three programs; Ctrl-C there (or closing Gazebo) stops them all.

```bash
scripts/start_sim.sh         # add --headless to skip the Gazebo window
```

Wait for `READY`, then run `hello_flight` in a second tab (below).
Don't close the Gazebo window while flying: it stops the simulation.

Or run each piece in its own terminal (useful for debugging):

```bash
scripts/sim_gazebo.sh        # 1. Gazebo with the Iris drone (add --headless to skip the window)
scripts/sim_ardupilot.sh     # 2. ArduCopter SITL + MAVProxy console/map (--headless for no windows, -w to reset params)
scripts/mavros.sh            # 3. MAVROS on tcp:5762
```

Then, in a fourth terminal:

```bash
source setup/env.sh
ros2 run drone_mission hello_flight
```

The drone should switch to GUIDED, arm (after ~30 s of SITL pre-arm checks), take off
to 2 m, fly 3 m East, hold, and land. First verified 2026-11-12: reached the target with 0.27 m
error and landed at x = 3.0 m. Logs from that run are in `logs/test_*.log`.

Run the scripts from this folder (`cd ~/Projects/Robotics/drone_mission` first): they use relative paths.

**Speed on this laptop:** with the Gazebo window open, the sim runs at ~0.3x real time
(check with `gz topic -e -n 1 -t /stats | grep real_time`), so everything looks slow-motion.
`hello_flight` multiplies its waits by `timeout_scale` (default 4) to cope. Headless runs
are much faster. If a run fails after arming, the node switches the drone to LAND.
MAVProxy's map window fails to load ("map not ready"); the console works and the map isn't needed.

Note: the default `iris_runway.sdf` drone (`iris_with_gimbal`) already has a gimbal camera
publishing on a Gazebo topic; that's a starting point for the marker work.

## Frames

MAVROS local frame is ENU: x = East, y = North, z = Up, metres from the arming point.
ArduPilot itself works in NED; MAVROS converts.

## Next steps

1. `waypoint_navigator` node: fly a list of (x, y, z) waypoints with an arrival tolerance.
2. Add a forward camera to the Iris model and an ArUco marker to a custom world in `sim/`.
3. `marker_approach` node: OpenCV ArUco pose + proportional control on yaw, height and range.
4. 10-run experiment with logging; plot paths and per-waypoint error.

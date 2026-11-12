# drone_mission

Course project: a quadcopter flies 3–4 waypoints, then finds a printed ArUco marker and holds
1 m in front of it for 5 s, then lands. Phase 1 is simulation (this repo); phase 2 runs the same
ROS 2 nodes on the real drone.

The real drone is the T27 GPS-Denied Drone (github.com/ahancock516/T27-GPS-Denied-Drone):
Raspberry Pi 5 + Aero Selfie H743 running **ArduCopter 4.6.3** (not PX4), forward-facing Pi
Camera 3 Wide. The sim matches that flight stack, so use ArduPilot concepts (GUIDED mode, not PX4
OFFBOARD). The original proposal said PX4; that was changed on purpose.

## Stack (Ubuntu 26.04 only)

- ROS 2 Lyrical in `/opt/ros/lyrical`, MAVROS (ROS 2), `ros_gz` → Gazebo Jetty (vendor packages)
- ArduCopter SITL `Copter-4.6.3` in `~/ardupilot`, Python venv `~/venv-ardupilot`
- ArduPilot Gazebo plugin in `~/ardupilot_gazebo` (pinned commit in `setup/01_install_system.sh`)
- This repo's ROS 2 package: `ros2_ws/src/drone_mission` (ament_python, built with `--symlink-install`)
- `source setup/env.sh` sets up ROS, Gazebo paths and ArduPilot tools in any shell

## Setting up a new machine

1. Clone `https://github.com/StevenFAU/Drone_Navigation` (any path works, e.g. `~/Projects/Robotics/Drone_Navigation`).
2. `setup/01_install_system.sh` needs sudo. Start it in a **Terminal-panel tab** (run_in_terminal),
   not Bash, so the user can type their password. Tell the user right away: sudo's password prompt
   times out after ~5 minutes. The apt step is long, and sudo may ask again later; watch the tab.
   Tee its output to `logs/01_install.log`.
   - If sudo fails with `The "no new privileges" flag is set`, the app's terminal is sandboxed.
     Open a normal Ubuntu terminal window instead with
     `setsid ptyxis --new-window -- bash -c 'cd <repo> && setup/01_install_system.sh 2>&1 | tee logs/01_install.log; exec bash' &`.
     Ptyxis is started through D-Bus, so the window doesn't inherit the flag. Then tell the user
     to type their password in that window, and read `logs/01_install.log` to follow progress.
3. `setup/02_build_sim.sh` needs no sudo. Run it in the background with output to
   `logs/02_build.log`. It takes ~10–20 min. `JOBS=4` by default; lower it if RAM is tight.
4. Verify:
   - Start `scripts/start_sim.sh --headless` in a Terminal tab and wait for `READY`.
   - Then run `source setup/env.sh && ros2 run drone_mission hello_flight --ros-args -p timeout_scale:=1.0`.
   - Success ends with "landed and disarmed".
   - Stop the sim with Ctrl-C in its tab.
5. Then show the user the GUI version: `scripts/start_sim.sh` (with the Gazebo window).

## Lessons from the first machine

- **Closing the Gazebo window stops the sim.** SITL then freezes and MAVROS loses the heartbeat;
  nodes wait forever at "waiting for MAVROS connection". Restart with `scripts/start_sim.sh`.
- With the Gazebo GUI on a laptop iGPU the sim ran at ~0.3x real time
  (`gz topic -e -n 1 -t /stats | grep real_time`). `hello_flight` has `timeout_scale`
  (default 4) for this. Headless is much faster.
- `ros2 topic echo --once /mavros/state` can show a stale `connected: false`. Trust
  `logs/sim_mavros.log` ("CON: Got HEARTBEAT, connected") instead.
- SITL startup prints "GPS 1: was not found", "Accels inconsistent" and "Bad fix" for a while.
  These are normal warm-up pre-arm messages; arming succeeds once they clear.
- MAVProxy's map window fails to load ("map not ready"). It's harmless; the console works.
- When killing sim processes, don't `pkill -f` a pattern that also appears in your own Bash
  command line (it kills your shell). Look up the PIDs and kill those. The SITL `arducopter`
  process runs in its own process group and can survive its terminal tab closing.
- ArduPilot's install script tried to compile wxPython from source on 26.04 and stalled. The
  install script now uses Ubuntu's `python3-wxgtk4.0` and `SKIP_AP_GRAPHIC_ENV=1` instead.
- Frames: MAVROS local frame is ENU (x East, y North, z Up), metres from the arming point.
  ArduPilot GUIDED velocity commands time out after ~3 s, so stream setpoints at ~10 Hz.

## Next steps (project plan)

1. `waypoint_navigator` node: an ordered list of (x, y, z) waypoints, an arrival tolerance, advance on arrival.
2. Sim world in `sim/worlds` with an ArUco marker. Use a fixed forward camera on the Iris to
   match the real drone (camera ~15 cm forward of the IMU, looking along body x).
   The default `iris_with_gimbal` has a gimbal camera to start from.
3. `marker_approach` node: OpenCV ArUco + `solvePnP`, with proportional control on yaw, height
   and range, sending body-frame velocity setpoints. Hold 1 m for 5 s, then land.
4. 10-run experiment: log the true pose from Gazebo, then plot the paths and a per-waypoint error table.

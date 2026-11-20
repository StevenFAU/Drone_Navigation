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
   If the repo is moved after building, delete `ros2_ws/{build,install,log}` and rebuild with
   `colcon build --symlink-install` (the install has absolute paths).
2. `setup/01_install_system.sh` needs sudo. Start it in a **Terminal-panel tab** (run_in_terminal),
   not Bash, so the user can type their password. Tell the user right away: sudo's password prompt
   times out after ~5 minutes. The apt step is long, and sudo may ask again later; watch the tab.
   Tee its output to `logs/01_install.log`.
   - If sudo fails with `The "no new privileges" flag is set`, the app's terminal is sandboxed.
     Open a normal Ubuntu terminal window instead with
     `setsid ptyxis --new-window -- bash -c 'cd <repo> && setup/01_install_system.sh 2>&1 | tee logs/01_install.log; exec bash' &`.
     Ptyxis is started through D-Bus, so the window doesn't inherit the flag, **but only if Ptyxis
     is already running**; otherwise it starts as a child of the sandboxed shell and still has the
     flag (check `grep NoNewPrivs /proc/<pid>/status`). Reliable on every machine:
     `systemd-run --user --collect -- ptyxis --new-window -- bash -c '<same command>'`. Then tell the user
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
- A uv/pyenv `python3` earlier on PATH than `/usr/bin/python3` makes the plugin's CMake fail with
  `No module named 'catkin_pkg'`; `02_build_sim.sh` now passes `-DPython3_EXECUTABLE=/usr/bin/python3`.
- `start_sim.sh` puts each program in its own process group, so killing the script's group doesn't
  stop them. Stop it with `kill -TERM <start_sim pid>` (its trap cleans up all of them). SIGINT
  only works from its own terminal: started in the background from a non-interactive shell
  (`setsid ... &`), bash ignores SIGINT. Don't find the PID with `pgrep -f`/`awk` on a pattern
  that is also in your own command line (it matched and killed my shell twice). A leftover sim makes the next one fail oddly (second Gazebo feeding the old SITL:
  "Gyro rate 70Hz", mag/GPS pre-arm errors, stuck at "waiting for mode GUIDED"). Before starting a
  sim, check `ps -eo pid,cmd | grep -E "[a]rducopter|[g]z-sim|[m]avros_node"` and kill by PID.
- Frames: MAVROS local frame is ENU (x East, y North, z Up), metres from the EKF origin: where ArduPilot first got a fix, **not** the latest arming point. It
  persists across flights in one SITL session, so mission nodes must record the takeoff position
  and offset by it (`waypoint_navigator` does).
  ArduPilot GUIDED velocity commands time out after ~3 s, so stream setpoints at ~10 Hz.
- Lyrical's rclpy logger has `.warning()` but no `.warn()`; `.warn` raised AttributeError inside
  the abort handler, so failed missions never switched to LAND. Use `.warning()`.
- Big images (1280x720 rgb8, 2.7 MB) over best-effort DDS lost fragments: 10 Hz with >1 s gaps
  instead of 15 Hz. The tracker subscribes reliable, depth 1 (`image_reliable` param).
- `solvePnP` on a far, grazing marker can return NaN; one NaN poisoned the filtered marker pose
  for good (drone sat still until lost_timeout). The tracker now rejects non-finite solves and
  normal flips (planar-pose ambiguity).
- ros_gz_bridge's Pose_V -> TFMessage loses the model names (empty child_frame_id); ground truth
  uses PosePublisher plugins per model (`/model/<name>/pose`) bridged as PoseStamped.
- WindEffects ignores model-level `<enable_wind>` for included models: `sim/models/iris_base_wind`
  is iris_with_standoffs with `enable_wind` on base_link. Force scaling 0.75 = ~4 deg lean at
  4.5 m/s. In 3.5 m/s gusts ArduPilot's own position hold wanders ±0.2 m (measured with a fixed
  target), so ±0.10 m marker-hold tolerances need wind under ~2.5 m/s.
- A drone landed in gusty wind can fail "Gyros inconsistent" and reject the next takeoff; the
  experiment runner restarts the sim for every run.
- SITL rangefinder type 100 ("SIM") gives "No Data" with Gazebo physics; `--gps-denied` uses the
  simulated analog sonar (`RNGFND1_TYPE 1`, pin 0) like ArduPilot's own optical-flow test.
- Sim options (`--noise`, `--gps-denied`) are SITL param files; `start_sim.sh` always passes `-w`
  so they never stick in the saved parameters.

- Real drone access: Pi 5 user `group7` (key login), `ssh group7@10.43.0.42` over the Ethernet
  cable after `nmcli con up drone-link` (laptop shares internet). raspberrypi.local flips between
  10.43.0.42 and the Pi's fixed 192.168.2.5 (unroutable from the laptop): use the IP. Pi Wi-Fi hotspot `raspberrypi` = 10.42.0.1.
- Drone ROS 2 uses ROS_DOMAIN_ID 27 (sim = 0) so a laptop sim can never command the real drone.
- On the Pi, an old docker-compose v2.23 in ~/.docker/cli-plugins shadowed the system v5 and
  failed against the daemon; it was renamed `docker-compose.v2.23-old`.
- Pi 5 camera needs Raspberry Pi's libcamera fork (pisp pipeline), built in robot/Dockerfile.
- The Pi throttled (vcgencmd get_throttled 0xe0008) with no fan; fan fitted 2026-11-13.
- MAVROS 2 with all plugins kept a Pi core at 100%; robot/config/mavros_plugins.yaml allowlists 12 (28%).
- Jazzy's Ubuntu 24.04 OpenCV is 4.6 (no cv2.aruco.ArucoDetector): marker_tracker.make_detector()
  falls back to the old API. Test drone-side code in the `mission` container (it mounts the package;
  dn_mavros has the image's build-time copy).
- MAV_CMD_SET_EKF_SOURCE_SET (42007) switches EKF sources without arming; not persistent.
- Drone containers must all run as root: Fast DDS shared-memory segments in /dev/shm are owned by
  their creator, so a non-root container hangs waiting for camera frames. The entrypoint chowns
  /logs to LOGS_OWNER (1000:1000) when a `mission` run ends instead.
- The drone's ELRS receiver runs in MAVLink mode on TELEM1 (SERIAL1, 460800): RC arrives as
  RC_CHANNELS_OVERRIDE from sysid 255, so `/mavros/rc/in` is empty; read `/uas1/mavlink_source`
  (tools/bench/rc_raw.py). `SERIAL1_OPTIONS` had to be 0 (not 8/swap). Details: docs/real_drone.md.
- Calibration photos come from the ROS stream (`python3 -m drone_mission.grab_frames`), not
  rpicam-still: dn_camera holds the camera. Print files: robot/print/ (print at 100%).

## Status (project plan)

Done: waypoint_navigator, ArUco world + forward camera, marker_approach (P velocity approach,
position-target final hold), `mission` (full run), ground truth + wind + noise, experiment
tools (`tools/run_experiment.py`, `tools/analyze_experiment.py`, results in `results/`),
GPS-denied sim mode, pilot-takeover detection, camera calibration tool. See README.

Phase 2 (real T27) set up 2026-11-13: drone surveyed, flight-controller params changed
(table + backups in docs/real_drone.md, robot/backup/), ROS 2 Jazzy Docker stack in robot/,
indoor mission config (heading_frame, ekf_source_set 2) tested 2/2 in the GPS-denied sim.
Next: camera calibration + camera_offset measurement (stage 0), then staged flights.

2026-11-16 (stage 0, partly done): camera calibrated (rational, RMS 0.18 px) and installed,
marker range passed at 1/2/3 m; camera_offset measured 2026-11-20 ([0.14, 0, 0]). 2026-11-20: offset synced,
radio re-paired, switch, stick and radio-loss (disarms in 0.8 s) checks passed. Left: floor mat,
then stage 1 (manual LOITER hover). Details and next steps: docs/stage0_progress_2026-11-16.md; voice-cued bench
helpers in tools/bench/.

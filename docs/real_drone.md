# Moving to the real T27 drone

The mission nodes talk only to MAVROS and a camera topic, so the same code runs on the T27's
Raspberry Pi 5. What changes is everything around it: how the drone knows where it is, the
camera driver and calibration, and safety. This is the bring-up plan, smallest risk first.

## What the T27 has (from its repo, `ahancock516/T27-GPS-Denied-Drone`)

| Item | T27 | Matters because |
| --- | --- | --- |
| Flight controller | Aero Selfie H743, **ArduCopter 4.6.3** | same version as the sim |
| Companion | Raspberry Pi 5 (8 GB) | runs MAVROS + these nodes |
| Camera | Pi Camera 3 Wide, forward | marker detection; **needs calibrating** (their `camera.yaml` is a placeholder: fx = 600, zero distortion) |
| Flow + range | Mico Air MTF-02P over MAVLink (`FLOW_TYPE 5`, `RNGFND1_TYPE 10`) | the only indoor position source |
| GPS / compass | HGLRC M100 Pro on a mast | outdoors / ground truth |
| EKF sources | set 1: GPS position + velocity, baro height, compass yaw. Set 2: **flow velocity only** | GUIDED needs a position estimate |
| VINS-Fusion | runs on the Pi (ROS 1 Noetic, Docker), **not fed to ArduPilot** (`VISO_TYPE 0`) | was evaluated offline only |
| RC | Radiomaster Pocket, ELRS; mode switch on channel 7: LOITER / ALT_HOLD / STABILIZE | pilot takeover |

## Position estimate: decide indoors vs outdoors

- **Outdoors with GPS**: use EKF source set 1 as configured. The sim's default mode matches this.
- **Indoors (GPS-denied)**: the MTF-02P gives velocity (flow) and height (rangefinder); the
  EKF dead-reckons position from flow. Set source set 1 (or switch to set 2 in flight) to
  `EK3_SRC1_POSXY 0, EK3_SRC1_VELXY 5, EK3_SRC1_VELZ 0, EK3_SRC1_POSZ 1` (or `2` to use the
  rangefinder for height), plus a rangefinder-friendly floor (textured, not glossy) and good
  light. `scripts/start_sim.sh --gps-denied` simulates exactly this; see the README for how the
  mission performs in it. The marker approach doesn't depend on drift (it is relative to the
  marker); waypoints do.
- VINS-Fusion could later replace flow (`VISO_TYPE 1`, MAVROS `vision_pose`), but it is ROS 1
  and needs a ROS 1 to ROS 2 bridge or a port. Not needed for this project.

## Software on the Pi

- ROS 2 Lyrical needs Ubuntu 26.04. On the Pi 5, either install Ubuntu 24.04 + **ROS 2 Jazzy**,
  or run Jazzy in Docker (the T27 already uses Docker). The nodes are plain `rclpy` + MAVROS +
  OpenCV and need no changes for Jazzy.
- MAVROS to the flight controller over the TELEM serial port, e.g.
  `ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600` (match `SERIALn_BAUD`).
- Camera: `camera_ros` (libcamera) publishes `/camera/image_raw` and `/camera/camera_info`.
  Run it at **1280x720** (same as the sim and the calibration), then pass the topics:
  `-p image_topic:=/camera/image_raw -p camera_info_topic:=/camera/camera_info`.
  The tracker subscribes with *reliable* QoS (big frames over best-effort lost ~30% in the sim).
  If `ros2 topic info -v /camera/image_raw` shows the driver publishes best-effort only, add
  `-p image_reliable:=false`, otherwise no frames arrive.
- Measure where the camera sits relative to the flight controller (forward, left, up in
  metres) and pass `-p camera_offset:="[0.15, 0.0, -0.02]"` with your numbers.
- Calibrate the camera with `tools/calibrate_camera.py` (instructions at the top of the file)
  and give `camera_ros` the YAML it writes.

## Safety setup (before any autonomous flight)

- **Pilot takeover**: keep the RC on and a pilot on the mode switch. Flipping it to LOITER or
  ALT_HOLD takes control back; the nodes notice the mode change and stop sending anything.
- Geofence: `FENCE_ENABLE 1`, `FENCE_TYPE 7`, `FENCE_RADIUS` and `FENCE_ALT_MAX` a little larger
  than the mission, `FENCE_ACTION 1` (RTL) or `2` (LAND indoors).
- Slow everything down for the first flights: `WPNAV_SPEED 100` (cm/s), and the node
  parameters `max_speed:=0.3`, `max_climb:=0.2`.
- Battery failsafe (`BATT_FS_LOW_ACT`), RC failsafe (`FS_THR_ENABLE`), and a netted or
  tethered area with nobody in front of the drone.

## Test stages (each one must pass before the next)

| Stage | What | Pass when |
| --- | --- | --- |
| 0. Bench, props off | MAVROS connects; `/mavros/state` shows the mode; `camera_ros` runs at 1280x720; calibrate; `tools/calibrate_camera.py --test` on photos of the printed marker at 1, 2, 3 m | range within ~3 cm at 1 m and ~5% at 3 m |
| 1. Manual hover | Pilot flies LOITER (indoors: with the flow source set); watch `/mavros/local_position/pose` while hovering 60 s | drift under ~0.5 m, no EKF warnings |
| 2. First GUIDED | `ros2 run drone_mission hello_flight --ros-args -p altitude:=1.0 -p target_x:=1.0` | takes off, moves 1 m, lands; pilot takeover tested once |
| 3. Marker only | Marker on a stand at 1.5 m height, drone 3 m in front: `marker_approach` with `target_range:=1.5`, then `1.0` | holds 5 s, lands |
| 4. Waypoints | A 1.5 m square with `waypoint_navigator` | each waypoint within ~0.3 m (eyeball / tape marks) |
| 5. Full mission | `mission` with waypoints sized to the room | complete run; then repeat for the real-world experiment |

## When to start

Stages 0 and 1 don't depend on any more code: start them now, in parallel with the sim work.
Calibration and the flow-drift check answer the two biggest real-world unknowns (range
accuracy, and whether indoor waypoints are good enough). Start flying stage 2 once stage 0 has
passed and the GPS-denied sim run is satisfactory, and leave at least two weeks before the
final demo for stages 2 to 5; real flights tend to need parameter tuning between sessions.

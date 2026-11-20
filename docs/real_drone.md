# Moving to the real T27 drone

The mission nodes talk only to MAVROS and a camera topic, so the same code runs on the T27's
Raspberry Pi 5. What changes is everything around it: how the drone knows where it is, the
camera driver and calibration, and safety. This is the bring-up plan, smallest risk first.

## What the T27 has (surveyed on the drone, 2026-11-13)

| Item | T27 | Notes |
| --- | --- | --- |
| Flight controller | MicoAir743 (reports itself so; the T27 docs say Aero Selfie H743), ArduCopter 4.6.3 | same version as the sim; to the Pi on `/dev/ttyAMA10` at 921600 |
| Companion | Raspberry Pi 5 (8 GB), Pi OS (Debian 12) | ran at 77-85 °C and throttled with no fan; a fan was fitted 2026-11-13. Watch `vcgencmd measure_temp` / `get_throttled` |
| Camera | Pi Camera 3 Wide, forward | the T27 ran it at 640x480 with a placeholder calibration (fx 600, no distortion) |
| Flow + range | Mico Air MTF-02P (`FLOW_TYPE 5`, `RNGFND1_TYPE 10`) | rangefinder limit `RNGFND1_MAX_CM 200`: **stay under ~1.8 m indoors** |
| GPS / compass | HGLRC M100 Pro on a mast | 0 satellites indoors |
| EKF sources | set 1: GPS. Set 2: flow velocity + rangefinder height. Set 3: same as set 2 | chosen by the **RC channel 6 switch** (`RC6_OPTION 90`), or by the mission (`ekf_source_set`) |
| RC | Radiomaster Pocket (ELRS 3.5.6, Link Mode MAVLink) -> SpeedyBee Nano 2.4 GHz RX (ELRS 3.5.1) on **TELEM1 = SERIAL1**, MAVLink at 460800 | ch 5 = arm, ch 6 = EKF source set (1000 = set 1 GPS, 1500 = set 2, 2000 = set 3), ch 7 = mode (1000 = LOITER, 1500 = ALT_HOLD, 2000 = STABILIZE). See "Radio link" below |
| Battery | 4S, 3300 mAh | low 13.97 V -> LAND, critical 13.71 V |

With source set 2 selected on the bench, the EKF gives a local position at 50 Hz (checked;
pre-arm then only complains that the radio is off). With set 1 indoors there is no position
at all, so GUIDED can't work: the indoor config selects set 2 itself.

## Flight-controller changes made for this project

All 1275 original parameters are in `robot/backup/t27_params_original.parm` (and the result in
`t27_params_after.parm`). To undo, load the original file in Mission Planner / QGroundControl.

| Parameter | Was | Now | Why |
| --- | --- | --- | --- |
| `EK3_SRC3_*` | all 0 (no sources) | same as set 2 | an RC6 switch position with no position, velocity or height source is a crash |
| `FLTMODE1`, `FLTMODE2` | 1 (ACRO) | 5 (LOITER) | match the T27 docs; ACRO is a poor takeover mode |
| `FS_THR_ENABLE` | 0 | 3 (LAND) | RC link loss did nothing |
| `FS_GCS_ENABLE` | 0 | 5 (LAND) | if the Pi/MAVROS dies mid-mission, land. `FS_OPTIONS 16` (already set) lets a pilot flying manually continue |
| `FENCE_ALT_MAX` | 30.5 m | 2.5 m | indoor ceiling; **raise it before flying outdoors** |
| `WPNAV_SPEED` / `_UP` / `_DN` | 1000 / 250 / 150 cm/s | 100 / 100 / 75 | slow first autonomous flights |
| `SERIAL1_OPTIONS` | 8 (swap TX/RX) | 0 | 2026-11-20: with the swap the FC heard nothing from the ELRS receiver on TELEM1 |

### Radio link (set up 2026-11-20)

The T27 team's radio wasn't available, so the receiver was re-paired to our Radiomaster Pocket.
The receiver (SpeedyBee Nano 2.4 GHz, web UI at 10.0.0.1 via its Wi-Fi `ExpressLRS RX` /
`expresslrs`, opened from the radio: ExpressLRS > WiFi Connectivity > Enable Rx WiFi) was in
**AirPort mode at 9600 baud**, which carries no RC at all. Changed to: AirPort off, UART baud
460800, Serial Protocol MAVLINK (originals saved in `logs/elrs_rx/config.json` on the laptop).
The radio's ExpressLRS Link Mode is MAVLink.

In MAVLink mode the receiver sends RC as `RC_CHANNELS_OVERRIDE` from system 255 (`SYSID_MYGCS
255`, `RC_OVERRIDE_TIME 3`). ArduPilot reports those with `chancount 0`, so **`/mavros/rc/in` stays
empty**; read the switches from `/uas1/mavlink_source` instead (`tools/bench/rc_raw.py`,
`tools/bench/rc_watch.py`). The mission nodes detect pilot takeover from the flight mode, so they
don't need `/mavros/rc/in`.

Switch check (props off): ch 6 low/mid/high -> EKF set 1 (GPS) / 2 / 3; ch 7 low/mid/high ->
LOITER / ALT_HOLD / STABILIZE (modes confirmed in `/mavros/state`). Following "up first", the
up position read high (2000) on both switches. **Indoors keep ch 6 away from set 1.**

## Software on the Pi (ROS 2 Jazzy in Docker)

Pi OS has no ROS 2 packages, so everything runs in one Ubuntu 24.04 + Jazzy image
(`robot/Dockerfile`): MAVROS 2, `camera_ros` built on Raspberry Pi's libcamera fork (needed for
the Pi 5), and the `drone_mission` package. `robot/docker-compose.yml` runs:

- `mavros`: `/dev/ttyAMA10:921600` (starts at boot). Only the 12 plugins the nodes use are loaded
  (`robot/config/mavros_plugins.yaml`); with all ~50 it kept a CPU core at 100%.
- `camera`: Pi Camera 3 Wide (IMX708) at 1280x720, 15 fps, `rgb8`, on `/camera/image_raw` +
  `/camera/camera_info`; focus locked at 2 m. Calibration from `robot/config/camera.yaml`
  (**currently a nominal placeholder**; see stage 0) (starts at boot). Checked: the tracker gets
  all 15 frames/s and spends 15 ms per frame.
- `mission`: only on demand: `docker compose run --rm mission` runs the mission with
  `config/mission_indoor.yaml`, logging to `~/Drone_Navigation/logs/`

The containers use **ROS domain 27**, so they never mix with a sim on the laptop (domain 0).
The T27 team's ROS 1 containers (roscore, mavros, camera_mono, vinsfusion) are stopped, not
deleted: `robot/install.sh --revert` brings them back (only one stack can own the serial port
and the camera).

### Connecting the laptop

The laptop has one Wi-Fi radio. Keep it on RoboNet for internet and plug an Ethernet cable
into the Pi, then:

```bash
nmcli con up drone-link       # laptop profile: shares the laptop's internet with the Pi
ssh group7@10.43.0.42         # key login is set up
```

The Pi gets 10.43.0.42 over the cable (the laptop's DHCP keeps it). `raspberrypi.local` also
works but sometimes resolves to the Pi's fixed 192.168.2.5, which the laptop can't reach. Without the cable,
join the drone's own Wi-Fi `raspberrypi` (Pi at 10.42.0.1), losing internet on the laptop.

```bash
scripts/drone_sync.sh                       # copy code/config changes to the Pi
source scripts/drone_env.sh                 # this terminal now sees the drone's ROS 2 topics
ros2 topic echo /mavros/state --once
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Node and config changes need only a sync (the containers mount the package). Changes to
`robot/Dockerfile` need `ssh` + `cd ~/Drone_Navigation/robot && ./install.sh`.

## Indoor mission (`config/mission_indoor.yaml`)

- `heading_frame: true`: waypoints are metres **forward / left** of the way the drone faces
  when it arms (not East/North). Point the drone along the room.
- `ekf_source_set: 2`: selects flow + rangefinder before arming.
- 1.5 m square at 1.2-1.4 m, then search, approach to 1.0 m from the marker, hold 5 s, land.
  Marker ~3-4 m straight ahead of the takeoff spot, centre ~1.2 m high, facing the drone.
- Slow: 0.3 m/s approach, 0.2 m/s climb, 15 °/s search.
- Tested in `start_sim.sh --gps-denied` (see the README).

## Safety (before any autonomous flight)

- **Pilot on the RC** the whole time, mode switch within reach. Flipping to LOITER or ALT_HOLD
  takes over; the nodes notice and stop sending anything (they never fight the pilot).
- Check on the radio which switch positions give LOITER / ALT_HOLD / STABILIZE (ch 7) and
  which EKF source set (ch 6: indoors = set 2 or 3, never 1).
- Netted/clear area, nobody in front of the drone, props off for every bench step.
- Optical flow needs texture under the drone. The lab floor is smooth light-grey concrete that
  reflects the ceiling lights: lay a textured mat, rug or crossing strips of tape along the
  flight path, and check LOITER drift over it in stage 1.

## Test stages (each one must pass before the next)

| Stage | What | Pass when |
| --- | --- | --- |
| 0. Bench, props off | Done 2026-11-13: `./install.sh`, MAVROS connected, camera at 1280x720 / 15 Hz. To do: **calibrate** (print `robot/print/*.pdf` at 100%, steps at the top of `tools/calibrate_camera.py`, photos via `drone_mission.grab_frames`), measure `camera_offset`, marker photos at 1, 2, 3 m with `--test` | range within ~3 cm at 1 m and ~5% at 3 m |
| 1. Manual hover | Pilot flies LOITER with source set 2; watch `ros2 topic echo /mavros/local_position/pose` while hovering 60 s | drift under ~0.5 m, no EKF warnings |
| 2. First GUIDED | `docker compose run --rm mission ros2 run drone_mission hello_flight --ros-args -p altitude:=1.0 -p target_x:=1.0 -p ekf_source_set:=2 -p heading_frame:=true` (1 m forward) | takes off, moves 1 m, lands; pilot takeover tested once |
| 3. Marker only | marker 3 m ahead at 1.2 m: `... marker_approach --ros-args --params-file /ws/src/drone_mission/config/mission_indoor.yaml -p target_range:=1.5`, then `1.0` | holds 5 s, lands |
| 4. Waypoints | `... waypoint_navigator --ros-args --params-file /ws/src/drone_mission/config/mission_indoor.yaml` | each waypoint within ~0.3 m (tape marks) |
| 5. Full mission | `docker compose run --rm mission` | complete run; then repeat for the real-world experiment |

(`...` = `docker compose run --rm mission ros2 run drone_mission`, from `~/Drone_Navigation/robot`.)

## When to start

Stage 0 can start as soon as a chessboard and a marker are printed. Stage 1 needs only a pilot.
Leave at least two weeks before the final demo for stages 2 to 5; real flights tend to need
parameter tuning between sessions.

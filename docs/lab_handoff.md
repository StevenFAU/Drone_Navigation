# Lab handoff: T27 stage 0 (bench, props off)

Written 2026-11-16 for a session at the lab with a laptop. Background: `CLAUDE.md`,
`docs/real_drone.md`. Everything below runs **on the drone's Pi over SSH**, so the laptop only
needs an SSH client (Linux, macOS or Windows all have `ssh`). Props off for all of it.

## What's already done

- The Pi runs this repo's ROS 2 Jazzy stack in Docker (`dn_mavros`, `dn_camera`, both start at
  boot). MAVROS connects to ArduCopter 4.6.3. The camera runs at 1280x720 / 15 Hz, focus locked at 2 m.
- Flight-controller safety/indoor parameters set (table in `docs/real_drone.md`; originals in
  `robot/backup/`).
- The indoor mission (`config/mission_indoor.yaml`) passed 2/2 in the GPS-denied sim.
- The camera calibration is still a **placeholder**: that's the main job today.

## Bring

- Both PDFs from `robot/print/`, printed at **100% / actual size** on US Letter. Check the
  100 mm bar on each page with a ruler; if it's off, reprint (don't use that print).
- Something flat and rigid for the chessboard (clipboard, foam board), tape, a tape measure,
  a ruler.
- A textured mat, rug or tape for the floor (optical flow needs texture; the lab floor is smooth).

## 1. Connect to the Pi

**Any laptop:** join the drone's Wi-Fi `raspberrypi` (password: the one you used on 2026-11-13;
not written in the repo). The Pi is **10.42.0.1**. The laptop has no internet while on it.

```bash
ssh group7@10.42.0.1        # asks for the Pi password (same person who flashed the SD card knows it)
```

**Linux laptop with an Ethernet cable** (keeps Wi-Fi internet): create the cable profile once, then
the Pi is 10.43.0.x (it was 10.43.0.42 on the lab desktop):

```bash
nmcli con add type ethernet ifname <your-ethernet-device> con-name drone-link ipv4.method shared ipv4.addresses 10.43.0.1/24 ipv6.method link-local connection.autoconnect no
nmcli con up drone-link
```

Optional, so you don't type the password every time: `ssh-copy-id group7@10.42.0.1` (Linux/macOS).

**Update the Pi's copy of the code first** (it's from 2026-11-13; two fixes since: the
calibration tool works with the Pi's OpenCV, and `tools/` is mounted in the container). From
the repo on the laptop, **before** leaving home (needs internet for `git pull`):

```bash
git clone https://github.com/StevenFAU/Drone_Navigation.git   # or: cd Drone_Navigation && git pull
```

then at the lab, connected to the Pi (Linux/macOS, needs `rsync`):

```bash
cd Drone_Navigation && DRONE=group7@10.42.0.1 scripts/drone_sync.sh
```

On Windows (no rsync), copy the two changed files instead:

```bash
scp tools/calibrate_camera.py group7@10.42.0.1:Drone_Navigation/tools/
scp robot/docker-compose.yml group7@10.42.0.1:Drone_Navigation/robot/
```

Check: `ssh group7@10.42.0.1 grep -c Dictionary_get Drone_Navigation/tools/calibrate_camera.py`
prints `1`.

## 2. Check the Pi and the fan

```bash
vcgencmd measure_temp; vcgencmd get_throttled
docker ps --format '{{.Names}} {{.Status}}'      # dn_mavros and dn_camera should be Up
```

It ran at 78-80 °C with the fan fitted on 2026-11-13; it slows itself down at 80-85 °C
(`get_throttled` not `0x0`). Check the fan spins. The official Pi 5 Active Cooler plugs into the
small 4-pin FAN socket next to the USB ports (not the GPIO pins), and its thermal pads must touch
the chips. With the fan working it should sit well under 70 °C.

All commands below are run from:

```bash
cd ~/Drone_Navigation/robot
```

## 3. Calibrate the camera

Take 25 photos, one every 2 s (it prints each one). Hold the chessboard 0.4-1.5 m from the
camera. Between shots, move it all over the image, **especially the corners and edges**, and tilt
it up to ~45°. Keep the whole board in view and hold it still when a photo is taken.

```bash
docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/calib --count 25 --every 2
```

Calibrate on the Pi (takes a minute):

```bash
docker compose run --rm mission python3 /tools/calibrate_camera.py /logs/calib --square 0.025 --out /logs/camera.yaml --show
```

Good result: **RMS under ~0.5 px** (it says which lens model it picked). Over 1 px: delete blurry
photos from `~/Drone_Navigation/logs/calib/` and run it again, or retake. `--show` writes corner
overlays and an undistorted preview to `~/Drone_Navigation/logs/camera_check/` (straight edges
should look straight).

Install it and restart the camera:

```bash
cp ~/Drone_Navigation/logs/camera.yaml config/camera.yaml && docker compose restart camera
```

## 4. Check marker range against a tape measure

Stand the printed marker so its centre is level with the camera, facing it, **1.00 m from the
lens** (measure lens to paper). Grab one photo (5 s to step out of the frame) and measure:

```bash
docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/marker_1m --count 1 --start-delay 5
docker compose run --rm mission python3 /tools/calibrate_camera.py --test /config/camera.yaml /logs/marker_1m/frame_000.jpg --marker-size 0.20
```

Repeat at 2 m and 3 m (`marker_2m`, `marker_3m`). Pass: within ~3 cm at 1 m, ~5% at 3 m.
If every reading is off by the same percentage, measure the printed marker's black square: the
200 mm must be right (it's `--marker-size`, and `marker_size` in `config/mission_indoor.yaml`).

## 5. Measure the camera position

From the flight controller's centre to the camera lens, in metres: **forward**, **left** (negative
if it's to the right) and **up** (negative if below). Write the three numbers down; they go into
`camera_offset` in `ros2_ws/src/drone_mission/config/mission_indoor.yaml` (currently a guess:
`[0.15, 0.0, -0.02]`).

## 6. Check the radio switches

Radio on, props off. In one terminal watch the RC channels, in another the flight mode:

```bash
docker compose run --rm mission ros2 topic echo /mavros/rc/in --field channels
docker compose run --rm mission ros2 topic echo /mavros/state --field mode
```

In `channels`, the 6th number is channel 6 and the 7th is channel 7 (microseconds, ~1000-2000).

- **Channel 6 = EKF source set**: below ~1300 = set 1 (GPS), ~1300-1700 = set 2 (flow), above
  ~1700 = set 3 (flow, same as 2). Note which switch position gives which. **Indoors: never set 1.**
- **Channel 7 = flight mode**: flip through every position and note the mode shown (expect
  LOITER / ALT_HOLD / STABILIZE).

## 7. Floor

Lay the textured mat / tape along where the drone will fly (about 2.5 m x 2.5 m plus the path to
the marker 3-4 m ahead).

## Bring back

Write these down (or paste them to Claude):

- Fan working? Temperature and `get_throttled` after 10 minutes.
- Calibration: lens model, RMS, number of photos used.
- Marker range readings at 1 / 2 / 3 m vs the tape.
- `camera_offset` forward / left / up.
- Channel 6 positions -> source set; channel 7 positions -> mode.

The new `camera.yaml` stays on the Pi (`~/Drone_Navigation/robot/config/`; `drone_sync.sh`
never overwrites it). To put it in the repo: `scp group7@10.42.0.1:Drone_Navigation/robot/config/camera.yaml robot/config/`
and commit, or ask Claude to fetch it next time it's connected.

## Notes for a Claude session on the laptop

- Repo: `github.com/StevenFAU/Drone_Navigation`, read `CLAUDE.md` and `docs/real_drone.md` first.
- Pi: user `group7`, Debian 12, the stack is `~/Drone_Navigation/robot/docker-compose.yml`
  (ROS domain 27). Test drone-side code in the `mission` container (it mounts the package,
  `tools/` and `logs/`); containers must stay root (DDS shared memory).
- Don't change flight-controller parameters, arm, or run a flight node (mission, hello_flight,
  marker_approach, waypoint_navigator) without the user's explicit go-ahead: stage 0 is bench only.
- The Pi's OpenCV is 4.6 (old ArUco API); `marker_tracker.make_detector()` handles both.

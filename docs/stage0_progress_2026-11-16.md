# Stage 0 progress, 2026-11-16 (bench, props off)

Session at the lab following `docs/lab_handoff.md`. Continue tomorrow from **"Still to do"**.

## Connection (this laptop)

- Ethernet cable to the Pi, then `nmcli con up drone-link` (profile created today on the USB
  adapter `enx9c69d34078eb`, shares the laptop's internet). Pi = **10.43.0.42**.
- The profile does not reconnect by itself: if the cable is unplugged, run `nmcli con up drone-link` again.
- Key login from this laptop is set up: `ssh group7@10.43.0.42` (no password).
- This folder came from `Drone_Navigation_repo.zip` (commit `afae116`); on 2026-11-20 it was turned
  into a git checkout of `github.com/StevenFAU/Drone_Navigation` and this work committed on branch `stage0-bench`.

## Done

| Step | Result |
| --- | --- |
| Code sync to Pi | `afae116` synced with `scripts/drone_sync.sh`; `Dictionary_get` check = 1, `/tools` mounted |
| Containers | `dn_mavros` + `dn_camera` up, MAVROS `connected: true` (only pre-arm: "RC not found", radio off) |
| Temperature | 36.7 °C at boot, 42.2 °C after ~1.7 h, 39.5 °C later; `get_throttled` = `0x0` throughout |
| Fan | Black blower fan at the back, wired to drone power (not the Pi's 4-pin FAN socket, so the Pi doesn't detect it). Keeps the Pi ~40 °C. |
| Camera calibration | **rational_polynomial, RMS 0.180 px, 56 photos** (3 sets). fx 824.4, fy 818.7, cx 646.0, cy 374.6. Horizontal FOV ~76° (camera_ros uses the IMX708's cropped 1536x864 mode, so not the lens's full 102°). |
| Calibration installed | Pi: `~/Drone_Navigation/robot/config/camera.yaml` (old placeholder kept as `camera.yaml.placeholder`), camera restarted, `/camera/camera_info` shows it. Laptop: copied to `robot/config/camera.yaml` here. |
| camera_offset (2026-11-20) | **[0.14, 0.0, 0.0]** in `mission_indoor.yaml`, read from tape photos: forward 13.4 cm FC centre -> camera board + ~5 mm to the lens; left ~0.3 cm (0); up estimated 0 +-1.5 cm (lens and FC both ~2 cm above the bottom plate). Not yet synced to the Pi. |
| Marker range (200 mm print) | tape 1.00 m -> **1.012 m**, 2.00 m -> **2.004 m**, 3.00 m -> **2.998 m** (forward distance). **Pass.** Print size is correct; `marker_size: 0.20` stays. |

Calibration notes: the photos only covered the centre and right of the image (nothing in the
top band, the left third or the far corners). Cross-check: fitted without the right-edge photos,
the rational model predicted them to 0.23 px (fisheye model: 0.51 px), so the rational model was
kept. Photos on the Pi: `~/Drone_Navigation/logs/calib`, `calib2`, `calib3`, merged in
`calib_all`; 2 blurred ones in `calib_rejected`. Early 1 m marker shots read 1.24 / 0.93 / 0.96 m
because the distance wasn't measured consistently; with the tape laid on the floor from the
drone, all three distances passed.

## Still to do

1. **Props off.** In today's photos all four props were still on. Remove them before the radio check
   (channel 5 is the arm switch).
2. **Radio pairing (blocker, 2026-11-20).** A Radiomaster Pocket (probably not the T27 team's own)
   would not pair with the drone's ELRS receiver: FC sends RC_CHANNELS with 0 channels, receiver LED
   blinking, Bind from the radio did nothing. The receiver is most likely locked to the T27 team's
   radio by a binding phrase. Fix: use the T27 team's radio, or get their binding phrase. Any radio
   used must also send arm on ch5 and 3-position switches on ch6 (EKF set) and ch7 (mode).
   Camera_offset was synced to the Pi on 2026-11-20 (Pi ran at 60 °C that day, top plate off).
3. **Radio switches** (after pairing). Radio on, then flip one switch at a time; the laptop announces which channel
   and what it does:
   `python3 -I tools/bench/rc_watch_voice.py` (first `scp tools/bench/rc_watch.py group7@10.43.0.42:Drone_Navigation/logs/`).
   Record which physical switch/position gives which value. Expected from the FC parameters:
   - ch5 = arm/disarm (`RC5_OPTION 154`)
   - ch6 = EKF source set (`RC6_OPTION 90`): low = set 1 GPS, mid = set 2 flow, high = set 3 flow. **Indoors never set 1.**
   - ch7 = flight mode (`FLTMODE_CH 7`): FLTMODE1/2 = LOITER, 3 = STABILIZE, 4 = ALT_HOLD, 5/6 = STABILIZE.
     A 3-position switch (1000/1500/2000) should give LOITER / ALT_HOLD / STABILIZE.
     LOITER may be refused while ch6 is on set 1 (no position indoors); try with set 2.
4. Optional: refine camera_offset `up` by measuring lens-centre and FC-board heights above the table.
5. **Floor.** Textured mat / tape over ~2.5 x 2.5 m plus the path to the marker.
6. ~~Commit~~ done 2026-11-20: this folder is now a git checkout (branch `stage0-bench`).

## Helper scripts (tools/bench/, made today)

- `cued_capture.py [/logs/calibN]`: calibration photos with spoken MOVE / HOLD STILL cues (laptop `spd-say`).
- `marker_shot.sh 1m|2m|3m`: spoken countdown, one marker photo, range test, reads the result aloud.
- `rc_watch.py` (runs on the Pi in the `mission` container, read-only) + `rc_watch_voice.py`
  (laptop): announces every switch change, EKF set, flight mode and arming; logs to `tools/bench/rc_check.log`.

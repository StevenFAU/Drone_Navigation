#!/bin/bash
# Usage: marker_shot.sh <tag, e.g. 1m>  -- voice-cued single marker photo + range test on the Pi
TAG=$1
spd-say -r -10 "Step out of the picture. Photo in about 10 seconds." &
OUT=$(ssh group7@10.43.0.42 "cd ~/Drone_Navigation/robot && rm -rf ../logs/marker_$TAG && docker compose run --rm mission python3 -m drone_mission.grab_frames --out /logs/marker_$TAG --count 1 --start-delay 5 >/dev/null 2>&1 && docker compose run --rm mission python3 /tools/calibrate_camera.py --test /config/camera.yaml /logs/marker_$TAG/frame_000.jpg --marker-size 0.20 2>&1 | grep -v Container")
echo "$OUT"
D=$(echo "$OUT" | grep -oP 'distance \K[0-9.]+' | head -1)
if [ -n "$D" ]; then
  CM=$(python3 -c "print(round($D*100))")
  spd-say -r -10 "Photo taken. Measured $CM centimetres."
else
  spd-say -r -10 "Photo taken. No marker found."
fi

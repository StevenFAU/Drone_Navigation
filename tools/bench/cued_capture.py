#!/usr/bin/env python3
"""Run grab_frames on the Pi and speak MOVE / HOLD STILL cues on the laptop for each photo."""
import subprocess, sys, threading, time

EVERY = 7.0        # seconds between photos
HOLD = 2.5         # say "hold still" this long before each photo
START = 12.0       # seconds before the first photo
OUT = sys.argv[1] if len(sys.argv) > 1 else '/logs/calib3'

# (spoken text, screen text)
POS = [
    ('top left. Aim below and right of the board', 'TOP-LEFT      aim below + right of board'),
    ('top left again, tilted', 'TOP-LEFT      again, drone tilted'),
    ('top middle. Aim straight below the board', 'TOP-MIDDLE    aim straight below board'),
    ('top middle again, tilted', 'TOP-MIDDLE    again, drone tilted'),
    ('top right. Aim below and left of the board', 'TOP-RIGHT     aim below + left of board'),
    ('top right again, tilted', 'TOP-RIGHT     again, drone tilted'),
    ('far left. Aim to the right of the board, same height', 'FAR-LEFT      aim right of board, same height'),
    ('far left again, tilted', 'FAR-LEFT      again, drone tilted'),
    ('bottom left. Aim above and right of the board', 'BOTTOM-LEFT   aim above + right of board'),
    ('bottom left again, tilted', 'BOTTOM-LEFT   again, drone tilted'),
    ('bottom right. Aim above and left of the board', 'BOTTOM-RIGHT  aim above + left of board'),
    ('bottom right again, tilted', 'BOTTOM-RIGHT  again, drone tilted'),
]
N = len(POS)


def say(text, screen=None):
    print(f'\n>>> {screen or text}', flush=True)
    subprocess.Popen(['spd-say', '-r', '-10', '-C'], stderr=subprocess.DEVNULL).wait()  # cancel any queued speech
    subprocess.Popen(['spd-say', '-r', '-10', text], stderr=subprocess.DEVNULL)


timers = []


def schedule(delay, fn, *a):
    t = threading.Timer(max(0.0, delay), fn, a)
    t.daemon = True
    t.start()
    timers.append(t)


def cue_next(k, until_photo):
    """Photo k (0-based) will be taken in until_photo seconds."""
    spoken, screen = POS[k]
    say(f'Move. Photo {k + 1}: {spoken}', f'MOVE   photo {k + 1}/{N}: {screen}')
    schedule(until_photo - HOLD, say, 'Hold still', f'HOLD STILL   (photo {k + 1})')


cmd = ['ssh', 'group7@10.43.0.42',
       f'cd ~/Drone_Navigation/robot && docker compose run --rm mission python3 -m drone_mission.grab_frames '
       f'--out {OUT} --count {N} --every {EVERY} --start-delay {START}']
p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
say('Get ready. Starting the camera.')
for line in p.stdout:
    line = line.rstrip()
    if 'Container' in line:
        continue
    print(line, flush=True)
    if 'First photo in' in line:
        cue_next(0, START)
    elif line.strip().startswith(tuple(f'{i}/{N}' for i in range(1, N + 1))):
        k = int(line.split('/')[0])
        subprocess.Popen(['canberra-gtk-play', '-i', 'camera-shutter'], stderr=subprocess.DEVNULL)
        if k < N:
            cue_next(k, EVERY)
        else:
            say('Done. All photos taken.', 'DONE - all photos taken')
p.wait()
time.sleep(3)
sys.exit(p.returncode)

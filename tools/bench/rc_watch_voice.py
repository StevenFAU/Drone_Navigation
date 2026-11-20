#!/usr/bin/env python3
"""Run rc_watch.py on the Pi, show its lines, speak each change, log to rc_check.log."""
import os, re, subprocess, sys, time

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rc_check.log")
cmd = ['ssh', 'group7@10.43.0.42',
       'cd ~/Drone_Navigation/robot && docker compose run --rm mission python3 /logs/rc_watch.py']


def say(t):
    subprocess.run(['spd-say', '-C'], stderr=subprocess.DEVNULL)
    subprocess.Popen(['spd-say', '-r', '-10', t], stderr=subprocess.DEVNULL)


say('Radio check running. Props must be off.')
waiting_said = False
seen = set()
with open(LOG, 'a') as log:
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in p.stdout:
        line = line.rstrip()
        if 'Container' in line:
            continue
        print(time.strftime('%H:%M:%S'), line, flush=True)
        log.write(time.strftime('%H:%M:%S ') + line + '\n'); log.flush()
        if line.startswith('waiting for RC') and not waiting_said:
            say('No radio signal yet. Turn the radio on.'); waiting_said = True
        m = re.match(r'EVENT (\w+)=(.*?)  \|', line)
        if not m:
            continue
        k, v = m.groups()
        if k.startswith('ch'):
            first = k not in seen
            seen.add(k)
            if first:            # radio just connected: don't read out every channel
                if k == 'ch8':
                    say('Radio connected. Flip one switch at a time.')
                continue
            msg = {'ch5': 'arm switch', 'ch6': 'E K F switch', 'ch7': 'mode switch'}.get(k, 'unused switch')
            say(f'Channel {k[2:]}, {msg}, {v}')
            time.sleep(2.2)
        elif k == 'ekf':
            say('E K F ' + v.replace('(', '').replace(')', ''))
        elif k == 'mode':
            say('Mode ' + v.replace('_', ' '))
        elif k == 'armed' and v == 'True':
            say('Warning. Armed. Disarm now.')

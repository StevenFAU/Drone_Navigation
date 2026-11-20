#!/usr/bin/env python3
"""Voice-guided bench check (props off, disarmed): stick directions, then radio-loss failsafe.
Runs on the laptop; streams tools/bench/rc_stream.py from the Pi (copy it to ~/Drone_Navigation/logs
first). Read-only: sends nothing to the drone. Results go to tools/bench/stick_check.log.

Expected raw values (Mode 2, AETR, ArduPilot RC2_REVERSED=1 on this drone): right stick forward ->
ch2 high, right stick right -> ch1 high, left stick up -> ch3 high, left stick right -> ch4 high.
"""
import os
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, 'stick_check.log')
CMD = ['ssh', 'group7@10.43.0.42',
       'cd ~/Drone_Navigation/robot && docker compose run --rm mission python3 /logs/rc_stream.py']

state = {'age': 999.0, 'ch': [0] * 8, 'mode': '-', 'armed': False}
texts = []
log = open(LOG, 'a')


def note(s):
    line = time.strftime('%H:%M:%S ') + s
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


def say(t, wait=True):
    note('SAY: ' + t)
    subprocess.run(['spd-say', '-r', '-10'] + (['-w'] if wait else []) + [t], stderr=subprocess.DEVNULL)


def reader(p):
    for line in p.stdout:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'RC' and len(parts) >= 12:
            state['age'] = float(parts[1])
            state['ch'] = [int(x) for x in parts[2:10]]
            state['mode'], state['armed'] = parts[10], parts[11] == '1'
        elif parts[0] == 'TEXT':
            texts.append((time.time(), line.strip()))
            note(line.strip())


def safe():
    """Block until the drone is disarmed with the arm switch low."""
    while state['armed'] or state['ch'][4] > 1500:
        say('Stop. Disarm now. Flip the arm switch back.')
        time.sleep(3)


def hold(secs):
    """Average channels over secs (only while RC is fresh)."""
    end, acc, n = time.time() + secs, [0] * 8, 0
    while time.time() < end:
        safe()
        if state['age'] < 0.5:
            acc = [a + c for a, c in zip(acc, state['ch'])]
            n += 1
        time.sleep(0.1)
    return [a // n for a in acc] if n else None


def main():
    p = subprocess.Popen(CMD, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    threading.Thread(target=reader, args=(p,), daemon=True).start()
    say('Starting. Props must be off.')
    t0 = time.time()
    while state['age'] > 0.5 and time.time() - t0 < 60:
        time.sleep(0.5)
    if state['age'] > 0.5:
        say('No radio data. Check the radio is on.')
        return
    safe()

    say('Let go of both sticks. Left stick fully down.')
    base = hold(3)
    note(f'centre: {base}')
    tests = [('Push the right stick fully forward, and hold.', 1, 'pitch'),
             ('Push the right stick fully right, and hold.', 0, 'roll'),
             ('Push the left stick fully up, and hold. The drone is disarmed.', 2, 'throttle'),
             ('Push the left stick fully right, and hold.', 3, 'yaw')]
    results = []
    for prompt, idx, name in tests:
        say(prompt)
        time.sleep(1.0)
        v = hold(2.5)
        val = v[idx] if v else 0
        ok = val > 1800
        results.append((name, val, ok))
        note(f'{name}: ch{idx + 1} = {val} -> {"OK" if ok else "WRONG (expected > 1800)"}')
        say(f'{name} {"OK" if ok else "wrong direction"}. Let go.' + (' Left stick back down.' if name == 'throttle' else ''))
        time.sleep(1.5)

    say('Now switch the radio off.')
    n0 = len(texts)
    t0 = time.time()
    while state['age'] < 2 and time.time() - t0 < 60:
        time.sleep(0.2)
    lost_at = time.time()
    note(f'radio data stopped (age {state["age"]:.1f} s)')
    time.sleep(8)
    fs = [t for _, t in texts[n0:]]
    note('FC messages after radio off: ' + (' | '.join(fs) if fs else 'none'))
    failsafe_seen = any('ailsafe' in t or 'RC not found' in t for t in fs)
    say('Failsafe reported.' if failsafe_seen else 'No failsafe message yet.')
    say('Switch the radio back on.')
    t0 = time.time()
    while state['age'] > 0.5 and time.time() - t0 < 90:
        time.sleep(0.5)
    time.sleep(3)
    back = [t for _, t in texts[n0:]]
    note('FC messages until radio back: ' + (' | '.join(back) if back else 'none'))
    note('SUMMARY sticks: ' + ', '.join(f'{n}={v} {"OK" if ok else "WRONG"}' for n, v, ok in results)
         + f' | failsafe message: {"yes" if failsafe_seen else "no"} (radio off at {time.strftime("%H:%M:%S", time.localtime(lost_at))})')
    say('Check finished.')
    p.terminate()


if __name__ == '__main__':
    main()
